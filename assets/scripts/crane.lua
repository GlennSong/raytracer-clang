-- A SMALL CRANE on the crane rig (rigs.lua): the first model made to be stamped as a prefab (#191, step 7). The cab
-- turns on the mast, the boom lifts, the hook line runs out. Each mesh is built in its own node's space; each solid
-- piece has a collision shape on its node, so the boom's shape goes where the boom goes. Seen from 120 m it is the
-- mast and the boom only.
--
-- It is boxes on purpose: it is here to prove the model and prefab records, and is to be rebuilt with the modelling
-- tools (a lattice boom, glazed cab, a rope and a load) once those exist.
-- Try: ./build-viewer/rt_model_sheet crane --set cabin=40 --set boom=30 --set hook=1.5 --out build-viewer/sheets/crane.png
local rigs = require "rigs"
local m = model.new()
m:material("steel", material.new{ roughness = 0.55, metallic = 0.6 })
m:material("cab", material.new{ roughness = 0.35, metallic = 0.1 })
local yellow, grey = { 0.85, 0.62, 0.08 }, { 0.35, 0.36, 0.38 }
local far = m:level(120)

local function box(size, at, color) return mesh.paint(mesh.translate(mesh.box(size), at), color) end

m:add(box({ 2, 0.4, 2 }, { 0, 0.2, 0 }, grey))                 -- the base, on the root
m:shape("base", { box = { 2, 0.4, 2 }, pos = { 0, 0.2, 0 }, friction = 0.9 })
m:add(box({ 0.5, 4, 0.5 }, { 0, 2.4, 0 }, yellow), "steel")    -- the mast
m:add(box({ 0.5, 4.4, 0.5 }, { 0, 2.2, 0 }, yellow), "steel", { level = far })
m:shape("mast", { box = { 0.5, 4, 0.5 }, pos = { 0, 2.4, 0 } })
m:marker("exhaust", { pos = { -0.8, 0.4, -0.8 }, turn = { -90, 0, 0 } })

local joint = rigs.apply(m, "crane", {
  cabin = { pos = { 0, 4.4, 0 } },
  boom = { pos = { 0, 1, 0.6 } },
  hook = { pos = { 0, 0, 5 } },
})
joint.cabin:add(box({ 1.2, 1, 1.2 }, { 0, 0.5, 0 }, grey), "cab")
joint.cabin:shape("cabin", { box = { 1.2, 1, 1.2 }, pos = { 0, 0.5, 0 } })
joint.cabin:marker("driver", { pos = { 0, 0.4, 0.2 }, role = "seat", size = 0.5 })
joint.cabin:marker("lamp_top", { pos = { 0, 1.05, 0 }, turn = { -90, 0, 0 }, role = "lamp" })
joint.boom:add(box({ 0.3, 0.3, 5 }, { 0, 0, 2.5 }, yellow), "steel")
joint.boom:add(box({ 0.3, 0.3, 5 }, { 0, 0, 2.5 }, yellow), "steel", { level = far })
joint.boom:shape("boom", { box = { 0.3, 0.3, 5 }, pos = { 0, 0, 2.5 } })
joint.hook:add(box({ 0.3, 0.3, 0.3 }, { 0, -0.15, 0 }, grey), "steel")
joint.hook:marker("hook", { pos = { 0, -0.3, 0 }, turn = { 90, 0, 0 }, role = "hook" })
return m
