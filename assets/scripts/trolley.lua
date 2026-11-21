-- THINGS ON WHEELS THAT STAFF AND SHOPPERS PUSH (#191). Each caster is on a node that turns about its stem,
-- `caster_1` ..., as an office chair's are. None of these rolls yet: a prefab is fixed or not solid for now, and
-- rolling needs a body with weight (#311).
--
--   cart       a supermarket's wire shopping cart; the child's seat at the back folds down (`child_seat`)
--   baskets    a stack of hand baskets on a low castered base (the stack is still; a marker `take` on top)
--   books      a library's book truck: two sloping shelves of books, each side
--   luggage    a hotel's luggage cart: a carpeted deck under a brass hoop
--
-- It stands on y = 0 at its own origin; it is pushed toward +z (its handle is at -z).
local kit = require "model_kit"
local V = kit.V

local a = params{
  kind  = { "cart", "baskets", "books", "luggage" },
  paint = { "red", "blue", "green", "orange", "yellow" },
}
local col = kit.paint[a.paint]
local m = model.new()
local far = m:level(40)
local c = kit.maker()
local STEEL, BRASS = V(0.78, 0.79, 0.8), V(0.8, 0.62, 0.25)
local function casters(xs, zs, r)
  local n = 0
  for _, x in ipairs(xs) do for _, z in ipairs(zs) do
    n = n + 1
    local node = m:node("caster_" .. n, { pos = { x, 0, z }, motion = { turn = { 0, 1, 0 } } })
    local k = kit.maker()
    kit.caster(k, V(0, 0, 0), r)
    kit.put(m, node, k)
  end end
end
local W, H, D
if a.kind == "cart" then
  W, H, D = 0.55, 1.0, 0.95
  casters({ -0.22, 0.22 }, { -0.36, 0.38 }, 0.05)
  for _, x in ipairs{ -0.24, 0.24 } do c.pipe("metal", { { x, 0.13, 0.42 }, { x, 0.13, -0.38 }, { x, 0.95, -0.47 } }, 0.012, STEEL, { bend = 0.08, sides = 6 }) end
  c.rod("paint", V(-0.26, 0.97, -0.47), V(0.26, 0.97, -0.47), 0.018, col, 8)                -- the handle
  -- the basket: wires along and across, wider at the back than the front
  local function rim(y, shrink) return { { -0.25 + shrink, y, -0.4 }, { 0.25 - shrink, y, -0.4 }, { 0.2 - shrink, y, 0.45 }, { -0.2 + shrink, y, 0.45 } } end
  for i = 0, 4 do c.pipe("metal", rim(0.45 + i * 0.11, (4 - i) * 0.012), i == 4 and 0.008 or 0.004, STEEL, { closed = true, sides = 5 }) end
  for i = 0, 9 do
    local t = i / 9
    for _, s in ipairs{ -1, 1 } do c.rod("metal", V(s * (0.2 - 0.05 * t * 0 + 0.002), 0.45, -0.4 + 0.85 * t), V(s * (0.25 - 0.05 * t), 0.89, -0.4 + 0.85 * t), 0.003, STEEL, 4) end
    c.rod("metal", V(-0.2, 0.45, -0.4 + 0.85 * t), V(0.2, 0.45, -0.4 + 0.85 * t), 0.003, STEEL, 4)
  end
  for i = 0, 5 do local x = -0.2 + i * 0.08; c.rod("metal", V(x, 0.45, 0.45), V(x, 0.89, 0.45), 0.003, STEEL, 4); c.rod("metal", V(x * 1.2, 0.45, -0.4), V(x * 1.25, 0.89, -0.4), 0.003, STEEL, 4) end
  c.box("metal", V(0, 0.2, 0.0), V(0.44, 0.012, 0.7), 0.0, STEEL * 0.8, 1)                  -- the shelf under the basket
  m:marker("push", { pos = { 0, 0, -0.85 }, role = "stand" })
  m:marker("handle", { pos = { 0, 0.97, -0.47 }, role = "hold" })
  m:marker("load", { pos = { 0, 0.5, 0.05 } })
  local seat = m:node("child_seat", { pos = { 0, 0.7, -0.39 }, motion = { turn = { 1, 0, 0 }, min = 0, max = 75 } })
  local k = kit.maker()
  k.box("paint", V(0, -0.1, 0.006), V(0.4, 0.2, 0.008), 0.004, col, 1)
  kit.put(m, seat, k)
elseif a.kind == "baskets" then
  W, H, D = 0.5, 0.75, 0.38
  casters({ -0.2, 0.2 }, { -0.13, 0.13 }, 0.03)
  c.box("metal", V(0, 0.09, 0), V(W, 0.02, D), 0.006, STEEL * 0.8, 1)
  for i = 0, 7 do
    local y = 0.1 + i * 0.06
    c.box("paint", V(0, y + 0.1, 0), V(0.4, 0.2, 0.26), 0.03, col * 0.85, 2)                 -- a basket's body, sitting in the one below
    c.box("paint", V(0, y + 0.185, 0), V(0.46, 0.03, 0.32), 0.012, col, 1)                  -- and its rim, which is what shows
  end
  c.pipe("hard", { { -0.2, 0.72, 0 }, { -0.2, 0.78, 0 }, { 0.2, 0.78, 0 }, { 0.2, 0.72, 0 } }, 0.008, kit.BLACK, { bend = 0.03, sides = 6 })
  m:marker("take", { pos = { 0, 0.78, 0 }, role = "hold" })
elseif a.kind == "books" then
  W, H, D = 0.9, 1.05, 0.45
  casters({ -0.38, 0.38 }, { -0.17, 0.17 }, 0.05)
  for _, x in ipairs{ -0.44, 0.44 } do c.box("paint", V(x, 0.58, 0), V(0.02, 0.9, D), 0.008, col, 1) end
  local shades = { V(0.55, 0.15, 0.12), V(0.15, 0.25, 0.45), V(0.2, 0.4, 0.25), V(0.7, 0.6, 0.35), V(0.3, 0.2, 0.3), V(0.8, 0.78, 0.7) }
  for row = 0, 1 do
    local y = 0.22 + row * 0.42
    for _, s in ipairs{ -1, 1 } do
      c.tilted("paint", V(0, y, s * 0.11), V(0.86, 0.012, 0.22), 0.0, -s * 0.25, col)       -- a shelf sloping in toward the middle
      local x = -0.4
      local i = 0
      while x < 0.36 do
        local w = 0.025 + 0.012 * ((i * 5 + row * 3) % 4)
        c.tilted("hard", V(x + w / 2, y + 0.115, s * 0.1), V(w - 0.002, 0.2 + 0.02 * (i % 3), 0.16), 0.002, -s * 0.25, shades[(i + row * 2 + (s > 0 and 3 or 0)) % #shades + 1])
        x, i = x + w, i + 1
      end
    end
  end
  c.rod("metal", V(-0.47, 0.98, -0.2), V(-0.47, 0.98, 0.2), 0.012, STEEL, 8)
  m:marker("handle", { pos = { -0.47, 0.98, 0 }, role = "hold" })
else
  W, H, D = 0.6, 1.75, 1.1
  casters({ -0.25, 0.25 }, { -0.45, 0.45 }, 0.06)
  c.box("metal", V(0, 0.2, 0), V(W, 0.04, D), 0.012, BRASS, 1)
  c.box("fabric", V(0, 0.224, 0), V(W - 0.04, 0.01, D - 0.04), 0.004, V(0.5, 0.08, 0.1), 1)
  for _, z in ipairs{ -D / 2 + 0.03, D / 2 - 0.03 } do
    c.pipe("metal", { { -W / 2 + 0.03, 0.22, z }, { -W / 2 + 0.03, 1.72, z }, { W / 2 - 0.03, 1.72, z }, { W / 2 - 0.03, 0.22, z } }, 0.016, BRASS, { bend = 0.28, sides = 8 })
  end
  c.rod("metal", V(0, 1.72, -D / 2 + 0.03), V(0, 1.72, D / 2 - 0.03), 0.014, BRASS, 8)     -- the rail the hangers go on
  c.box("hard", V(0.02, 0.44, -0.2), V(0.4, 0.42, 0.22), 0.03, V(0.25, 0.18, 0.12), 2)     -- a case
  c.box("hard", V(-0.03, 0.39, 0.2), V(0.36, 0.32, 0.5), 0.04, V(0.12, 0.2, 0.35), 2)      -- and another
  m:marker("hang", { pos = { 0, 1.7, 0 }, role = "hold" })
  m:marker("load", { pos = { 0, 0.23, 0 } })
end
kit.put(m, m, c)
kit.far_box(m, m, "paint", { W, H * 0.7, D }, { 0, H * 0.45, 0 }, col, far)
m:shape("body", { box = { W, H, D }, pos = { 0, H / 2, 0 } })
return m
