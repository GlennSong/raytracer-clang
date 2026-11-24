-- SMALL THINGS THAT STAND ON A PAVEMENT OR A LOBBY FLOOR (#191):
--
--   sandwich_board   a shop's A-shaped board: its two leaves are hinged at the top, and the back one is on a
--                    node, `leaf`, that closes against the front one (30 degrees) to fold it away. A marker
--                    `words` on each face.
--   stanchions       queue posts joined by a rope that hangs between them (cable.hang): a bank's, a town hall's,
--                    a cinema's. Nothing moves. A marker `queue_N` beside each span.
--   flagpole         a pole with a flag on a node, `flag`, that slides from the foot to the top (a civic
--                    building's, a school's). Markers: `halyard` at hand height.
--
-- It stands on y = 0 at its own origin.
local kit = require "model_kit"
local cable = require "cable"
local V = kit.V

local a = params{
  kind   = { "sandwich_board", "stanchions", "flagpole" },
  count  = { 4, min = 2, max = 10, step = 1 },    -- stanchions: how many posts
  height = { 8.0, min = 4, max = 15 },            -- flagpole (metres)
  colour = { "red", "blue", "green", "yellow", "orange" },
}
local col = kit.paint[a.colour]
local m = model.new()
local far = m:level(60)
local c = kit.maker()

if a.kind == "sandwich_board" then
  local W, H, LEAN = 0.6, 1.0, 15
  local function leaf(k)                          -- a leaf hanging from the hinge at its top, in the hinge's space
    k.box("wood", V(0, -H / 2, 0), V(W, H, 0.02), 0.006, kit.WOOD[2], 1)
    k.box("paint", V(0, -H / 2 + 0.06, 0.011), V(W - 0.08, H - 0.24, 0.002), 0.0, V(0.08, 0.09, 0.09), 1)   -- the chalkboard
    for i = 0, 3 do k.box("ceramic", V(-0.05 + (i % 2) * 0.03, -0.25 - i * 0.14, 0.0125), V(0.3 - (i % 3) * 0.06, 0.035, 0.001), 0.0, kit.WHITE, 1) end   -- chalked lines
  end
  local front = m:node("front_leaf", { pos = { 0, H * math.cos(math.rad(LEAN)), 0 }, turn = { -LEAN, 0, 0 } })
  local f = kit.maker(); leaf(f); kit.put(m, front, f)
  front:marker("words_front", { pos = { 0, -H / 2, 0.02 }, role = "screen", size = 0.5 })
  local back = m:node("leaf", { pos = { 0, H * math.cos(math.rad(LEAN)), 0 }, turn = { LEAN, 180, 0 }, motion = { turn = { 1, 0, 0 }, min = 0, max = 2 * LEAN } })
  local b = kit.maker(); leaf(b); kit.put(m, back, b)
  back:marker("words_back", { pos = { 0, -H / 2, 0.02 }, role = "screen", size = 0.5 })
  m:shape("board", { box = { W, H, 0.5 }, pos = { 0, H / 2, 0 } })
  kit.far_box(m, m, "wood", { W, H, 0.3 }, { 0, H / 2, 0 }, kit.WOOD[2], far)
elseif a.kind == "stanchions" then
  local N, GAP, TOP = math.floor(a.count + 0.5), 1.5, 0.95
  local BRASS = V(0.8, 0.62, 0.25)
  for i = 1, N do
    local x = (i - (N + 1) / 2) * GAP
    c.turned("metal", V(x, 0, 0), { { 0, 0 }, { 0.16, 0 }, { 0.16, 0.02 }, { 0.04, 0.05 }, { 0.025, 0.07 }, { 0.025, TOP - 0.03 }, { 0.04, TOP - 0.02 }, { 0.04, TOP }, { 0, TOP + 0.02 } }, BRASS, 1, 16)
    m:shape("post_" .. i, { capsule = { radius = 0.04, height = TOP }, pos = { x, TOP / 2, 0 } })
    if i < N then
      local line = cable.hang({ x + 0.04, TOP - 0.06, 0 }, { x + GAP - 0.04, TOP - 0.06, 0 }, { sag = 0.22, steps = 14 })
      c.mesh("fabric", cable.rope(line, { radius = 0.018, strands = 3, pitch = 0.09, sides = 5, color = col }))
      m:marker("queue_" .. i, { pos = { x + GAP / 2, 0, 0.5 }, turn = { 0, 90, 0 }, role = "stand" })
      m:shape("rope_" .. i, { box = { GAP, 0.3, 0.04 }, pos = { x + GAP / 2, TOP - 0.2, 0 } })
    end
  end
  kit.put(m, m, c)
else
  local H = a.height
  c.turned("metal", V(0, 0, 0), { { 0, 0 }, { 0.2, 0 }, { 0.2, 0.05 }, { 0.07, 0.1 }, { 0.06, 0.4 }, { 0.03, H }, { 0, H } }, V(0.82, 0.83, 0.84), 1, 16)
  c.mesh("metal", mesh.translate(mesh.sphere(0.07), { 0, H + 0.05, 0 }), V(0.85, 0.7, 0.3))
  c.rod("fabric", V(0.065, 1.2, 0), V(0.035, H - 0.05, 0), 0.004, kit.LINEN, 4)            -- the halyard
  c.box("metal", V(0.07, 1.2, 0), V(0.03, 0.12, 0.03), 0.004, V(0.82, 0.83, 0.84), 1)      -- the cleat
  kit.put(m, m, c)
  m:shape("pole", { capsule = { radius = 0.06, height = H }, pos = { 0, H / 2, 0 } })
  m:marker("halyard", { pos = { 0.08, 1.2, 0 }, role = "hold" })
  kit.far_box(m, m, "metal", { 0.1, H, 0.1 }, { 0, H / 2, 0 }, V(0.82, 0.83, 0.84), far)
  local FW, FH = H * 0.22, H * 0.14
  local flag = m:node("flag", { pos = { 0.05, 1.5, 0 }, motion = { slide = { 0, 1, 0 }, min = 0, max = H - 1.6 - FH } })
  local k = kit.maker()
  -- a flag in a light wind: a cloth with a wave in it, three bands of colour
  for band = 0, 2 do
    local cloth = mesh.rounded_box({ FW, FH / 3, 0.006 }, 0.0)
    cloth = mesh.translate(cloth, { FW / 2, FH / 6 + band * FH / 3, 0 })
    k.mesh("fabric", cloth, ({ col, kit.LINEN, col })[band + 1])
  end
  local waved = {}
  for i, mm in ipairs(k.lists.fabric) do waved[i] = mesh.move_by(mm, function(_, p) return { p[1], p[2] - 0.12 * p[1] * p[1] / FW, 0.05 * FW * math.sin(p[1] / FW * 5) * (p[1] / FW) + p[3] } end) end
  k.lists.fabric = waved
  kit.put(m, flag, k)
  kit.far_box(m, flag, "fabric", { FW, FH, 0.01 }, { FW / 2, FH / 2, 0 }, col, far)
  flag:marker("flag", { pos = { FW / 2, FH / 2, 0 } })
end
m:marker("front", { pos = { 0, 0, 1.0 }, turn = { 0, 180, 0 }, role = "stand" })
return m
