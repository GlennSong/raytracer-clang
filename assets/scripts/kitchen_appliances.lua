-- KITCHEN APPLIANCES FOR A HOME, each with parts that move (#191):
--
--   oven          a cooker: its oven door drops open (`door`), five knobs turn (`knob_1` ...), a hob on top
--   dishwasher    its door drops open (`door`) and its two racks slide out over it (`rack_low`, `rack_high`)
--   toaster       its lever goes down (`lever`) and the toast with it (`toast`); as placed the toast is up
--   stand_mixer   its head tilts back (`head`) and its whisk turns (`whisk`, on the head)
--   rice_cooker   its lid lifts at the back (`lid`)
--
-- The oven and the dishwasher stand on the floor; the others on a worktop. Each has its front toward +z.
local kit = require "model_kit"
local V = kit.V

local a = params{
  kind   = { "oven", "dishwasher", "toaster", "stand_mixer", "rice_cooker" },
  finish = { "white", "steel", "black", "red" },
}
local col, part = kit.finish(a.finish)
local m = model.new()
local far = m:level(25)
local c = kit.maker()

-- a door hinged along its foot that drops open toward +z: w wide and h tall, standing on the node's origin
local function drop_door(name, at, w, h, window)
  local n = m:node(name, { pos = at, motion = { turn = { 1, 0, 0 }, min = 0, max = 90 } })
  local k = kit.maker()
  if window then
    for _, x in ipairs{ -w / 2 + 0.04, w / 2 - 0.04 } do k.box(part, V(x, h / 2, 0.02), V(0.08, h, 0.04), 0.006, col, 1) end
    k.box(part, V(0, 0.05, 0.02), V(w - 0.16, 0.1, 0.04), 0.006, col, 1)
    k.box(part, V(0, h - 0.07, 0.02), V(w - 0.16, 0.14, 0.04), 0.006, col, 1)
    k.box("glass", V(0, h / 2 - 0.02, 0.02), V(w - 0.16, h - 0.24, 0.012), 0.0, V(0.15, 0.17, 0.2), 1)
  else
    k.box(part, V(0, h / 2, 0.02), V(w, h, 0.04), 0.008, col, 1)
    k.box("hard", V(0, h / 2, -0.002), V(w - 0.06, h - 0.06, 0.004), 0.0, V(0.75, 0.76, 0.78), 1)   -- its steel inside
  end
  k.rod("metal", V(-w / 2 + 0.08, h - 0.06, 0.07), V(w / 2 - 0.08, h - 0.06, 0.07), 0.009, kit.CHROME, 8)
  for _, x in ipairs{ -w / 2 + 0.1, w / 2 - 0.1 } do k.rod("metal", V(x, h - 0.06, 0.04), V(x, h - 0.06, 0.07), 0.006, kit.CHROME, 6) end
  kit.put(m, n, k)
  n:shape(name, { box = { w, h, 0.04 }, pos = { 0, h / 2, 0.02 } })
  n:marker(name .. "_handle", { pos = { 0, h - 0.06, 0.08 }, role = "hold" })
end

if a.kind == "oven" then
  local W, H, D, T, FOOT, STRIP = 0.6, 0.9, 0.6, 0.03, 0.12, 0.12
  kit.carcass(c, part, W, H, D, T, col, FOOT)
  c.box(part, V(0, FOOT / 2, -0.01), V(W, FOOT, D - 0.02), 0.006, col * 0.9, 1)                     -- the drawer under the oven
  c.box(part, V(0, H - STRIP / 2, D / 2 - T / 2), V(W - 2 * T, STRIP, T), 0.0, col, 1)              -- the strip the knobs are on
  c.box("hard", V(0, H + 0.004, 0), V(W - 0.02, 0.008, D - 0.02), 0.0, kit.BLACK * 1.2, 1)          -- the hob's glass
  for _, p in ipairs{ { -0.14, -0.13, 0.09 }, { 0.14, -0.13, 0.07 }, { -0.14, 0.12, 0.07 }, { 0.14, 0.12, 0.09 } } do
    c.turned("hard", V(p[1], H + 0.008, p[2]), { { p[3] - 0.012, 0 }, { p[3], 0 }, { p[3], 0.002 }, { p[3] - 0.012, 0.002 } }, V(0.5, 0.12, 0.08), 1, 24)
  end
  for i = 1, 2 do c.box("metal", V(0, FOOT + (H - STRIP - FOOT) * i / 3, 0), V(W - 2 * T, 0.006, D - 0.1), 0.0, V(0.7, 0.71, 0.73), 1) end   -- the shelves
  c.box("light", V(0, H - STRIP - 0.05, -D / 2 + T + 0.01), V(0.06, 0.03, 0.01), 0.0, V(1, 0.85, 0.5), 1)
  kit.put(m, m, c)
  kit.far_box(m, m, part, { W, H, D }, { 0, H / 2, 0 }, col, far)
  m:shape("body", { box = { W, H, D }, pos = { 0, H / 2, 0 } })
  for i = 1, 5 do
    local n = m:node("knob_" .. i, { pos = { (i - 3) * 0.1, H - STRIP / 2, D / 2 }, motion = { turn = { 0, 0, -1 }, min = 0, max = 270 } })
    local k = kit.maker()
    k.mesh("hard", kit.facing(mesh.lathe{ profile = { { 0, 0 }, { 0.022, 0 }, { 0.019, 0.022 }, { 0, 0.022 } }, segments = 14, smooth = true }), kit.BLACK * 1.4)
    k.box("hard", V(0, 0.011, 0.023), V(0.004, 0.02, 0.002), 0.0, kit.WHITE, 1)                     -- the mark that shows where it points
    kit.put(m, n, k)
    n:marker("knob_" .. i, { pos = { 0, 0, 0.03 }, role = "hold" })
  end
  drop_door("door", { 0, FOOT, D / 2 }, W - 0.01, H - FOOT - STRIP, true)
  m:marker("hob", { pos = { 0, H + 0.01, 0 } })
elseif a.kind == "dishwasher" then
  local W, H, D, T, FOOT = 0.6, 0.85, 0.58, 0.03, 0.1
  kit.carcass(c, part, W, H, D, T, col, FOOT)
  c.box("hard", V(0, FOOT / 2, -0.03), V(W - 0.02, FOOT, D - 0.08), 0.0, kit.BLACK, 1)
  c.box("hard", V(0, (H + FOOT) / 2, -D / 2 + T + 0.002), V(W - 2 * T, H - FOOT - 2 * T, 0.004), 0.0, V(0.75, 0.76, 0.78), 1)
  kit.put(m, m, c)
  kit.far_box(m, m, part, { W, H, D }, { 0, H / 2, 0 }, col, far)
  m:shape("body", { box = { W, H, D }, pos = { 0, H / 2, 0 } })
  for _, r in ipairs{ { "rack_low", FOOT + T + 0.03 }, { "rack_high", FOOT + T + 0.38 } } do
    local n = m:node(r[1], { pos = { 0, r[2], 0 }, motion = { slide = { 0, 0, 1 }, min = 0, max = 0.46 } })
    local k = kit.maker()
    local w, d = W - 2 * T - 0.03, D - T - 0.08
    k.box("metal", V(0, 0.003, 0.01), V(w, 0.006, d), 0.0, V(0.8, 0.81, 0.83), 1)
    k.pipe("metal", { { -w / 2, 0.1, -d / 2 + 0.01 }, { w / 2, 0.1, -d / 2 + 0.01 }, { w / 2, 0.1, d / 2 + 0.01 }, { -w / 2, 0.1, d / 2 + 0.01 } }, 0.004, V(0.8, 0.81, 0.83), { closed = true, sides = 6 })
    for _, x in ipairs{ -w / 2, w / 2 } do for _, z in ipairs{ -d / 2 + 0.01, d / 2 + 0.01 } do k.rod("metal", V(x, 0, z), V(x, 0.1, z), 0.004, V(0.8, 0.81, 0.83), 6) end end
    if r[1] == "rack_low" then
      for i = 1, 8 do k.mesh("ceramic", mesh.translate(kit.facing(mesh.lathe{ profile = { { 0, 0 }, { 0.11, 0 }, { 0.115, 0.012 }, { 0, 0.006 } }, segments = 18, smooth = true }), { -0.08, 0.125, -0.17 + i * 0.04 }), kit.PORCELAIN) end
    else
      for i = 0, 5 do k.turned("ceramic", V(-0.16 + (i % 3) * 0.16, 0.008, -0.08 + (i // 3) * 0.16), { { 0, 0.08 }, { 0.035, 0.08 }, { 0.04, 0 }, { 0.036, 0 }, { 0.031, 0.075 }, { 0, 0.075 } }, kit.PORCELAIN, 1, 12) end   -- cups, upside down
    end
    kit.put(m, n, k)
    n:marker(r[1], { pos = { 0, 0.1, d / 2 + 0.02 }, role = "hold" })
  end
  drop_door("door", { 0, FOOT, D / 2 }, W - 0.01, H - FOOT - 0.01, false)
elseif a.kind == "toaster" then
  c.box(part, V(0, 0.105, 0), V(0.3, 0.17, 0.17), 0.03, col, 3)
  for _, x in ipairs{ -0.11, 0.11 } do c.box("hard", V(x, 0.01, 0), V(0.03, 0.02, 0.13), 0.004, kit.BLACK, 1) end
  for _, z in ipairs{ -0.035, 0.035 } do c.box("hard", V(0, 0.19, z), V(0.2, 0.003, 0.03), 0.0, kit.BLACK, 1) end   -- the slots
  c.box("hard", V(0.151, 0.1, 0), V(0.003, 0.11, 0.012), 0.0, kit.BLACK, 1)                                        -- the lever's slot
  kit.put(m, m, c)
  m:shape("body", { box = { 0.3, 0.19, 0.17 }, pos = { 0, 0.095, 0 } })
  local toast = m:node("toast", { pos = { 0, 0.12, 0 }, motion = { slide = { 0, -1, 0 }, min = 0, max = 0.075 } })
  local t = kit.maker()
  for _, z in ipairs{ -0.035, 0.035 } do t.box("hard", V(0, 0.06, z), V(0.11, 0.11, 0.014), 0.006, V(0.78, 0.56, 0.28), 1) end
  kit.put(m, toast, t)
  local lever = m:node("lever", { pos = { 0.158, 0.145, 0 }, motion = { slide = { 0, -1, 0 }, min = 0, max = 0.075 } })
  local l = kit.maker()
  l.box("hard", V(0, 0, 0), V(0.024, 0.016, 0.045), 0.005, kit.BLACK, 1)
  kit.put(m, lever, l)
  lever:marker("lever", { pos = { 0.015, 0, 0 }, role = "hold" })
elseif a.kind == "stand_mixer" then
  c.box(part, V(0, 0.02, 0.02), V(0.2, 0.04, 0.34), 0.015, col, 2)
  c.box(part, V(0, 0.16, -0.1), V(0.1, 0.26, 0.1), 0.03, col, 3)
  c.turned("metal", V(0, 0.04, 0.07), { { 0, 0 }, { 0.05, 0 }, { 0.1, 0.05 }, { 0.11, 0.14 }, { 0.105, 0.14 }, { 0.095, 0.055 }, { 0, 0.012 } }, V(0.8, 0.81, 0.83), 1, 24)
  kit.put(m, m, c)
  m:shape("body", { box = { 0.2, 0.3, 0.34 }, pos = { 0, 0.15, 0.02 } })
  local head = m:node("head", { pos = { 0, 0.29, -0.1 }, motion = { turn = { -1, 0, 0 }, min = 0, max = 45 } })
  local h = kit.maker()
  h.box(part, V(0, 0.04, 0.11), V(0.115, 0.11, 0.34), 0.045, col, 3)
  h.turned("metal", V(0, -0.03, 0.17), { { 0, 0 }, { 0.022, 0 }, { 0.022, 0.02 }, { 0, 0.02 } }, kit.CHROME, 1, 12)
  kit.put(m, head, h)
  head:marker("head", { pos = { 0, 0.06, 0.28 }, role = "hold" })
  local whisk = head:node("whisk", { pos = { 0, -0.03, 0.17 }, motion = { turn = { 0, 1, 0 } } })
  local w = kit.maker()
  w.rod("metal", V(0, 0, 0), V(0, -0.04, 0), 0.004, kit.CHROME, 6)
  for i = 0, 2 do
    local cs, sn = math.cos(i * math.pi / 3), math.sin(i * math.pi / 3)
    local loop = {}
    for _, p in ipairs{ { 0, -0.035 }, { 0.04, -0.08 }, { 0.032, -0.13 }, { 0, -0.15 }, { -0.032, -0.13 }, { -0.04, -0.08 } } do loop[#loop + 1] = { p[1] * cs, p[2], -p[1] * sn } end
    w.pipe("metal", loop, 0.0015, kit.CHROME, { closed = true, bend = 0.02, sides = 4 })
  end
  kit.put(m, whisk, w)
else
  c.turned(part, V(0, 0, 0), { { 0, 0 }, { 0.11, 0 }, { 0.13, 0.03 }, { 0.13, 0.17 }, { 0.115, 0.17 } }, col, 1, 28)
  c.turned("metal", V(0, 0, 0), { { 0.115, 0.17 }, { 0.11, 0.07 }, { 0, 0.065 } }, V(0.75, 0.76, 0.78), 1, 28)                  -- the pot inside
  c.turned("hard", V(0, 0, 0), { { 0, 0.118 }, { 0.111, 0.118 }, { 0.111, 0.12 }, { 0, 0.125 } }, V(0.97, 0.97, 0.95), 1, 20)    -- and the rice in it
  c.box("hard", V(0, 0.09, 0.122), V(0.085, 0.07, 0.02), 0.008, kit.BLACK, 1)
  c.box("light", V(0, 0.105, 0.133), V(0.02, 0.008, 0.002), 0.0, V(1, 0.5, 0.1), 1)
  kit.put(m, m, c)
  m:shape("body", { box = { 0.26, 0.17, 0.26 }, pos = { 0, 0.085, 0 } })
  m:marker("button", { pos = { 0, 0.08, 0.135 }, role = "press" })
  local lid = m:node("lid", { pos = { 0, 0.17, -0.125 }, motion = { turn = { -1, 0, 0 }, min = 0, max = 85 } })
  local k = kit.maker()
  k.turned(part, V(0, 0, 0.125), { { 0, 0 }, { 0.13, 0 }, { 0.13, 0.02 }, { 0.11, 0.05 }, { 0.05, 0.066 }, { 0, 0.068 } }, col, 1, 28)
  k.box("hard", V(0, 0.03, 0.25), V(0.05, 0.02, 0.02), 0.006, kit.BLACK, 1)
  kit.put(m, lid, k)
  lid:shape("lid", { box = { 0.26, 0.06, 0.26 }, pos = { 0, 0.03, 0.125 } })
  lid:marker("lid", { pos = { 0, 0.03, 0.26 }, role = "hold" })
end
m:marker("front", { pos = { 0, 0, 0.8 }, turn = { 0, 180, 0 }, role = "stand" })
return m
