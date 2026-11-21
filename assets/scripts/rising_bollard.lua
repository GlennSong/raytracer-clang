-- A BOLLARD THAT SINKS INTO THE ROAD (#191): a steel post with a lit band, in a ring set in the ground. The post is
-- on a node, `post`, that slides down until its top is level with the road. As placed it is up.
-- It stands on y = 0 at its own origin.
local kit = require "model_kit"
local V = kit.V

local a = params{
  height = { 0.8, min = 0.5, max = 1.1 },         -- how far it stands out of the ground (metres)
  radius = { 0.11, min = 0.07, max = 0.18 },
}
local H, R = a.height, a.radius
local STEEL = V(0.6, 0.62, 0.65)
local m = model.new()
local far = m:level(50)
local ring = kit.maker()
ring.turned("metal", V(0, 0, 0), { { R + 0.012, -0.01 }, { R + 0.07, -0.01 }, { R + 0.07, 0.006 }, { R + 0.012, 0.01 } }, STEEL * 0.8, 1, 24)
kit.put(m, m, ring)

local post = m:node("post", { pos = { 0, 0, 0 }, motion = { slide = { 0, 1, 0 }, min = -H, max = 0 } })
local k = kit.maker()
k.turned("metal", V(0, 0, 0), { { R, -0.05 }, { R, H - 0.02 }, { R - 0.02, H }, { 0.0, H } }, STEEL, 1, 24)
k.turned("light", V(0, 0, 0), { { R + 0.002, H - 0.16 }, { R + 0.002, H - 0.1 } }, V(1, 0.75, 0.2), 1, 24)   -- the lit band
k.turned("paint", V(0, 0, 0), { { R + 0.001, H - 0.3 }, { R + 0.001, H - 0.22 } }, V(0.95, 0.95, 0.9), 1, 24)   -- a reflective one
kit.put(m, post, k)
kit.far_box(m, post, "metal", { R * 1.8, H, R * 1.8 }, { 0, H / 2, 0 }, STEEL, far)
post:shape("post", { capsule = { radius = R, height = H - 2 * R }, pos = { 0, H / 2, 0 } })
post:marker("lamp", { pos = { 0, H - 0.13, 0 }, role = "lamp" })
return m
