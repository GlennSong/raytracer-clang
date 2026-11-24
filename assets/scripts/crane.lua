-- A SMALL CRANE on the crane rig (rigs.lua), made to be stamped as a prefab (prefabs.lua `crane`, `crane_working`;
-- #191). The cab turns on the mast, the boom lifts, the hook line runs out. Each mesh is built in its own node's
-- space; each solid piece has a collision shape on its node, so the boom's shape goes where the boom goes. Seen from
-- 120 m it is a plain mast and boom.
--
-- Built with the modelling tools: the mast and the boom are lattices (truss.lua), the cab is four walls with real
-- windows (openings.lua: a frame, glazing and a pane in each), and the base, the cab and the hook block have
-- bevelled edges (P:bevel). Still to come, each waiting on its system (docs/model-record-design.md, "The crane
-- Glenn asked for"): surfaces with texture (#309), a rope and a load that swing (#311), a cab to climb into.
-- Try: ./build-viewer/rt_model_sheet crane --set cabin=40 --set boom=30 --set hook=1.5 --out build-viewer/sheets/crane.png
local rigs = require "rigs"
local truss = require "truss"
local openings = require "openings"

local m = model.new()
m:material("steel", material.new{ roughness = 0.55, metallic = 0.6 })
m:material("cab", material.new{ roughness = 0.35, metallic = 0.1 })
m:material("glass", material.new{ roughness = 0.05, metallic = 0.0, opacity = 0.35 })
local yellow, grey, white, tint = { 0.85, 0.62, 0.08 }, { 0.35, 0.36, 0.38 }, { 0.9, 0.9, 0.88 }, { 0.55, 0.7, 0.75 }
local far = m:level(120)

local function box(size, at, color) return mesh.paint(mesh.translate(mesh.box(size), at), color) end
-- a quad mesh as one painted mesh
local function solid(P, color)
  local parts = {}
  for _, part in pairs(P:to_parts(25)) do parts[#parts + 1] = part end
  return mesh.paint(mesh.merge(parts), color)
end

-- THE BASE, on the root: a slab with its edges cut back
local base = poly.box{ 2, 0.4, 2 }
base:translate{ 0, 0.2, 0 }
base:bevel(0.04, 30, nil, 2)
m:add(solid(base, grey))
m:shape("base", { box = { 2, 0.4, 2 }, pos = { 0, 0.2, 0 }, friction = 0.9 })

-- THE MAST: a square lattice from the base to the cab (it runs straight up, so its section's "up" is z)
local mast = truss.build{ path = { { 0, 0.4, 0 }, { 0, 4.4, 0 } }, up = { 0, 0, 1 }, bay = 0.5, size = { 0.44, 0.44 },
                          chord = 0.03, brace = 0.015, nodes = true, color = yellow }
m:add(mast, "steel")
m:add(box({ 0.5, 4.4, 0.5 }, { 0, 2.2, 0 }, yellow), "steel", { level = far })
m:shape("mast", { box = { 0.5, 4, 0.5 }, pos = { 0, 2.4, 0 } })
m:marker("exhaust", { pos = { -0.8, 0.4, -0.8 }, turn = { -90, 0, 0 } })

local joint = rigs.apply(m, "crane", {
  cabin = { pos = { 0, 4.4, 0 } },
  boom = { pos = { 0, 1, 0.6 } },
  hook = { pos = { 0, 0, 5 } },
})

-- THE CAB: a floor, a roof and four walls 5 cm thick, 1.2 m square and 1 m tall. The front and both sides have a
-- window; the back is plain.
local T = 0.05
local walls, frames, panes = {}, {}, {}
local function wall(size, at, window)
  local w = poly.box(size)
  w:translate(at)
  if window then
    local made = openings.window(w, window)
    made.frame:bevel(0.004)
    frames[#frames + 1], panes[#panes + 1] = solid(made.frame, white), solid(made.pane, tint)
  end
  w:bevel(0.008)
  walls[#walls + 1] = solid(w, grey)
end
wall({ 1.2, T, 1.2 }, { 0, T / 2, 0 })                                    -- floor
wall({ 1.2, T, 1.2 }, { 0, 1 - T / 2, 0 })                                -- roof
wall({ 1.2, 1 - 2 * T, T }, { 0, 0.5, 0.6 - T / 2 },                      -- front: one wide light
     { at = { 0, 0.55, 0.6 }, normal = { 0, 0, 1 }, width = 0.9, height = 0.55, thickness = T, frame = 0.03,
       frame_depth = 0.04, set_back = 0.005, bars = { 1, 0 }, pane = 0.006 })
wall({ 1.2, 1 - 2 * T, T }, { 0, 0.5, -0.6 + T / 2 })                     -- back
for _, side in ipairs{ -1, 1 } do
  wall({ T, 1 - 2 * T, 1.2 - 2 * T }, { side * (0.6 - T / 2), 0.5, 0 },
       { at = { side * 0.6, 0.55, 0 }, normal = { side, 0, 0 }, width = 0.7, height = 0.5, thickness = T, frame = 0.03,
         frame_depth = 0.04, set_back = 0.005, pane = 0.006 })
end
joint.cabin:add(mesh.merge(walls), "cab")
joint.cabin:add(mesh.merge(frames), "cab")
joint.cabin:add(mesh.merge(panes), "glass")
joint.cabin:shape("cabin", { box = { 1.2, 1, 1.2 }, pos = { 0, 0.5, 0 } })
joint.cabin:marker("driver", { pos = { 0, 0.4, 0.2 }, role = "seat", size = 0.5 })
joint.cabin:marker("lamp_top", { pos = { 0, 1.05, 0 }, turn = { -90, 0, 0 }, role = "lamp" })

-- THE BOOM: a lattice 5 m long, tapering to its tip, in the boom's own space (it runs along +z)
local boom = truss.build{ path = { { 0, 0, 0 }, { 0, 0, 5 } }, bay = 0.5, size = { 0.34, 0.34 }, size_end = { 0.14, 0.14 },
                          chord = 0.022, brace = 0.012, nodes = true, color = yellow }
joint.boom:add(boom, "steel")
joint.boom:add(box({ 0.3, 0.3, 5 }, { 0, 0, 2.5 }, yellow), "steel", { level = far })
joint.boom:shape("boom", { box = { 0.3, 0.3, 5 }, pos = { 0, 0, 2.5 } })

-- THE HOOK BLOCK
local block = poly.box{ 0.3, 0.3, 0.3 }
block:translate{ 0, -0.15, 0 }
block:bevel(0.03, 30, nil, 2)
joint.hook:add(solid(block, grey), "steel")
joint.hook:marker("hook", { pos = { 0, -0.3, 0 }, turn = { 90, 0, 0 }, role = "hook" })
return m
