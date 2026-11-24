-- WHAT HANGS AT A HOME'S WINDOWS, drawn and opened (#191):
--
--   curtains   a pair on a rail. Each curtain is six folds, each on a node of its own (`left_1` ... `left_6`,
--              `right_1` ...; 1 is nearest the wall) that slides toward its side and stops where it is gathered.
--              Send them all outward at one speed and the curtains draw back (curtains_draw.lua does). As placed
--              they are closed. They stand on y = 0, the rail at `height`.
--   venetian   a blind of slats, each on a node (`slat_1` ...) that tilts either way. Its origin is the middle
--              of the sill under it.
--
-- Each faces +z, the room's side. (A roller blind is the projector screen's recipe at a window's size:
-- models.lua `roller_blind`.)
local kit = require "model_kit"
local V = kit.V

local a = params{
  kind   = { "curtains", "venetian" },
  width  = { 2.0, min = 0.6, max = 4.0 },         -- metres
  height = { 2.3, min = 0.6, max = 3.2 },         -- the rail's height, or the blind's drop
  fabric = kit.fabric_names,
}
local W, H = a.width, a.height
local m = model.new()
local far = m:level(50)
local c = kit.maker()

if a.kind == "curtains" then
  local cloth = kit.fabric(a.fabric)
  local N, GATHERED = 6, 0.05
  local w = W / 2 / N
  c.rod("metal", V(-W / 2 - 0.08, H, 0.06), V(W / 2 + 0.08, H, 0.06), 0.012, kit.STEEL, 10)
  for _, x in ipairs{ -W / 2 - 0.08, W / 2 + 0.08 } do
    c.mesh("metal", mesh.translate(kit.ball(0.024), { x, H, 0.06 }), kit.STEEL)
    c.rod("metal", V(x * 0.97, H, 0), V(x * 0.97, H, 0.06), 0.008, kit.STEEL, 6)
  end
  kit.put(m, m, c)
  for _, side in ipairs{ -1, 1 } do
    for i = 1, N do
      local name = (side < 0 and "left_" or "right_") .. i
      local n = m:node(name, { pos = { side * (W / 2 - (i - 0.5) * w), H - 0.02, 0.06 + (i % 2 == 0 and 0.014 or -0.014) },
                               motion = { slide = { side, 0, 0 }, min = 0, max = (i - 0.5) * (w - GATHERED) } })
      local k = kit.maker()
      k.box("fabric", V(0, -(H - 0.04) / 2, 0), V(w + 0.014, H - 0.04, 0.05), 0.022, cloth * (i % 2 == 0 and 1 or 0.88), 2)
      k.turned("metal", V(0, 0.005, 0), { { 0.012, 0 }, { 0.02, 0 }, { 0.02, 0.006 }, { 0.012, 0.006 } }, kit.STEEL, 1, 8)   -- its ring
      kit.put(m, n, k)
      kit.far_box(m, n, "fabric", { w, H - 0.04, 0.03 }, { 0, -(H - 0.04) / 2, 0 }, cloth, far)
      if i == N then n:marker(side < 0 and "pull_left" or "pull_right", { pos = { -side * w / 2, -H * 0.45, 0.04 }, role = "hold" }) end
    end
  end
else
  local col = V(0.93, 0.93, 0.9)
  local PITCH = 0.045
  local count = math.floor((H - 0.08) / PITCH)
  c.box("paint", V(0, H - 0.02, 0), V(W, 0.04, 0.05), 0.006, col, 1)
  c.box("paint", V(0, 0.012, 0), V(W, 0.022, 0.045), 0.006, col, 1)
  for _, x in ipairs{ -W / 2 + 0.12, W / 2 - 0.12 } do c.rod("hard", V(x, 0.02, 0), V(x, H - 0.03, 0), 0.0012, col * 0.8, 3) end
  c.rod("hard", V(-W / 2 + 0.05, H - 0.04, 0.03), V(-W / 2 + 0.05, H * 0.45, 0.03), 0.004, V(0.9, 0.9, 0.88), 6)   -- the wand that tilts them
  kit.put(m, m, c)
  kit.far_box(m, m, "paint", { W, H, 0.02 }, { 0, H / 2, 0 }, col, far)
  m:marker("wand", { pos = { -W / 2 + 0.05, H * 0.45, 0.03 }, role = "hold" })
  for i = 1, count do
    local n = m:node("slat_" .. i, { pos = { 0, 0.04 + (i - 0.5) * PITCH, 0 }, motion = { turn = { 1, 0, 0 }, min = -75, max = 75 } })
    local k = kit.maker()
    k.box("paint", V(0, 0, 0), V(W - 0.01, 0.002, 0.042), 0.0, col, 1)
    kit.put(m, n, k)
  end
end
return m
