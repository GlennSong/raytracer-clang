-- THINGS AT A PUBLIC COUNTER, A LOBBY OR A CORRIDOR, each with a part that moves (#191):
--
--   ballot_box          a box on a stand with a slot; a folded paper drops in (`paper`) and its lid lifts (`lid`)
--   room_safe           a small safe: its door opens (`door`) and the dial on it turns (`dial`)
--   fire_hose_cabinet   a red wall cabinet: a glazed door (`door`) and the hose reel inside swings out (`reel`,
--                       with the drum on it turning: `drum`)
--   reception_bell      a desk bell: its button is pressed down (`button`)
--   teller_window       a counter window with a blind that unrolls to close it (`blind_1` ... and `bar`) and a tray that
--                       slides under the glass (`tray`)
--   ticket_dispenser    a "take a number" stand: a ticket comes out of its mouth (`ticket`), a lit number above
--
-- Each has its front toward +z. The cabinet's origin is the foot of its back (put it on a wall); the others
-- stand on y = 0 (the safe, the bell and the window on a shelf or a counter).
local kit = require "model_kit"
local V = kit.V

local a = params{ kind = { "ballot_box", "room_safe", "fire_hose_cabinet", "reception_bell", "teller_window", "ticket_dispenser" } }
local STEEL, DARK, BRASS = kit.metal.steel, kit.paint.black, kit.metal.brass
local m = model.new()
local far = m:level(30)
local c = kit.maker()

if a.kind == "ballot_box" then
  local W, Y = 0.45, 0.75
  for _, x in ipairs{ -0.18, 0.18 } do for _, z in ipairs{ -0.18, 0.18 } do c.post("metal", V(x * 1.25, 0, z * 1.25), V(x, Y, z), 0.03, 0.03, STEEL) end end
  for _, x in ipairs{ -W / 2, W / 2 } do c.box("glass", V(x, Y + 0.2, 0), V(0.008, 0.4, W), 0.0, V(0.85, 0.9, 0.92), 1) end
  for _, z in ipairs{ -W / 2, W / 2 } do c.box("glass", V(0, Y + 0.2, z), V(W, 0.4, 0.008), 0.0, V(0.85, 0.9, 0.92), 1) end
  c.box("paint", V(0, Y, 0), V(W + 0.02, 0.02, W + 0.02), 0.004, DARK, 1)
  for i = 0, 5 do c.tilted("hard", V(-0.1 + (i % 3) * 0.1, Y + 0.03 + i * 0.012, -0.08 + (i % 2) * 0.14), V(0.15, 0.004, 0.1), 0.0, 0.2 * (i % 3 - 1), V(0.97, 0.97, 0.93)) end   -- the votes in it
  kit.put(m, m, c)
  kit.far_box(m, m, "glass", { W, 0.4, W }, { 0, Y + 0.2, 0 }, V(0.85, 0.9, 0.92), far)
  m:shape("box", { box = { W, Y + 0.42, W }, pos = { 0, (Y + 0.42) / 2, 0 } })
  local lid = m:node("lid", { pos = { 0, Y + 0.4, -W / 2 }, motion = { turn = { -1, 0, 0 }, min = 0, max = 80 } })
  local l = kit.maker()
  for _, x in ipairs{ -W / 4 - 0.045, W / 4 + 0.045 } do l.box("paint", V(x, 0.01, W / 2), V(W / 2 - 0.07, 0.02, W + 0.02), 0.004, DARK, 1) end
  for _, z in ipairs{ 0.105, W - 0.105 } do l.box("paint", V(0, 0.01, z), V(0.18, 0.02, 0.23), 0.004, DARK, 1) end     -- leaving a slot across the middle
  kit.put(m, lid, l)
  lid:marker("slot", { pos = { 0, 0.03, W / 2 }, role = "press" })
  local paper = m:node("paper", { pos = { 0, Y + 0.56, 0 }, motion = { slide = { 0, -1, 0 }, min = 0, max = 0.5 } })
  local p = kit.maker()
  p.box("hard", V(0, 0, 0), V(0.14, 0.1, 0.003), 0.0, V(0.97, 0.97, 0.93), 1)
  kit.put(m, paper, p)
elseif a.kind == "room_safe" then
  local W, H, D, T = 0.42, 0.3, 0.36, 0.025
  kit.carcass(c, "paint", W, H, D, T, DARK)
  c.box("fabric", V(0, T + 0.002, 0), V(W - 2 * T, 0.004, D - T), 0.0, kit.fabric("rust"), 1)
  c.box("hard", V(-0.06, T + 0.03, -0.02), V(0.16, 0.05, 0.1), 0.006, V(0.3, 0.2, 0.5), 1)             -- a passport and a box in it
  c.box("hard", V(0.1, T + 0.012, 0.02), V(0.09, 0.012, 0.125), 0.003, V(0.4, 0.1, 0.12), 1)
  kit.put(m, m, c)
  kit.far_box(m, m, "paint", { W, H, D }, { 0, H / 2, 0 }, DARK, far)
  m:shape("body", { box = { W, H, D }, pos = { 0, H / 2, 0 } })
  local door = m:node("door", { pos = { -W / 2, 0, D / 2 }, motion = { turn = { 0, -1, 0 }, min = 0, max = 110 } })
  local k = kit.maker()
  k.box("paint", V(W / 2, H / 2, 0.015), V(W - 0.004, H - 0.004, 0.03), 0.006, DARK * 1.2, 1)
  k.box("hard", V(W / 2 + 0.09, H / 2 + 0.04, 0.031), V(0.12, 0.09, 0.003), 0.0, kit.BLACK, 1)        -- the keypad
  k.box("light", V(W / 2 + 0.09, H / 2 + 0.07, 0.033), V(0.08, 0.016, 0.001), 0.0, V(0.3, 1, 0.4), 1)
  kit.put(m, door, k)
  door:shape("door", { box = { W, H, 0.03 }, pos = { W / 2, H / 2, 0.015 } })
  door:marker("keypad", { pos = { W / 2 + 0.09, H / 2 + 0.03, 0.035 }, role = "press" })
  local dial = door:node("dial", { pos = { W / 2 - 0.07, H / 2, 0.03 }, motion = { turn = { 0, 0, -1 } } })
  local d = kit.maker()
  d.mesh("metal", kit.facing(mesh.lathe{ profile = { { 0, 0 }, { 0.045, 0 }, { 0.04, 0.02 }, { 0, 0.02 } }, segments = 20, smooth = true }), STEEL)
  d.box("metal", V(0, 0, 0.03), V(0.09, 0.014, 0.014), 0.005, STEEL, 1)                                -- the bar to turn it by
  kit.put(m, dial, d)
  dial:marker("dial", { pos = { 0, 0, 0.04 }, role = "hold" })
elseif a.kind == "fire_hose_cabinet" then
  local W, H, D, T = 0.7, 0.8, 0.24, 0.02
  local RED = kit.paint.red
  kit.carcass(c, "paint", W, H, D, T, RED)
  kit.put(m, m, c)
  kit.far_box(m, m, "paint", { W, H, D }, { 0, H / 2, 0 }, RED, far)
  m:shape("body", { box = { W, H, D }, pos = { 0, H / 2, 0 } })
  local reel = m:node("reel", { pos = { W / 2 - T - 0.02, H / 2, D / 2 - 0.04 }, motion = { turn = { 0, 1, 0 }, min = 0, max = 100 } })
  local r = kit.maker()
  r.rod("paint", V(0, -0.3, 0), V(0, 0.3, 0), 0.012, RED, 6)
  r.rod("paint", V(0, 0, 0), V(-0.3, 0, -0.06), 0.012, RED, 6)                                         -- the arm the drum is on
  kit.put(m, reel, r)
  local drum = reel:node("drum", { pos = { -0.3, 0, -0.06 }, motion = { turn = { 0, 0, 1 } } })
  local d = kit.maker()
  for _, z in ipairs{ -0.045, 0.045 } do d.mesh("paint", mesh.translate(kit.facing(mesh.lathe{ profile = { { 0, -0.004 }, { 0.28, -0.004 }, { 0.28, 0.004 }, { 0, 0.004 } }, segments = 24 }), { 0, 0, z }), RED) end
  d.mesh("fabric", kit.facing(mesh.lathe{ profile = { { 0.08, -0.04 }, { 0.24, -0.04 }, { 0.24, 0.04 }, { 0.08, 0.04 } }, segments = 24 }), V(0.85, 0.82, 0.7))   -- the hose wound on it
  d.rod("metal", V(0.24, 0, 0.0), V(0.24, -0.16, 0.02), 0.014, BRASS, 6)                              -- its nozzle, hanging
  for i = 0, 3 do d.mesh("hard", mesh.rotate_z(mesh.translate(mesh.box{ 0.03, 0.1, 0.002 }, { 0, 0.2, 0.05 }), i * math.pi / 2), kit.WHITE) end
  kit.put(m, drum, d)
  drum:marker("nozzle", { pos = { 0.24, -0.16, 0.02 }, role = "hold" })
  local door = m:node("door", { pos = { -W / 2, 0, D / 2 }, motion = { turn = { 0, -1, 0 }, min = 0, max = 150 } })
  local k = kit.maker()
  for _, x in ipairs{ 0.03, W - 0.03 } do k.box("paint", V(x, H / 2, 0.01), V(0.06, H, 0.02), 0.004, RED, 1) end
  for _, y in ipairs{ 0.03, H - 0.03 } do k.box("paint", V(W / 2, y, 0.01), V(W - 0.12, 0.06, 0.02), 0.004, RED, 1) end
  k.box("glass", V(W / 2, H / 2, 0.01), V(W - 0.12, H - 0.12, 0.005), 0.0, V(0.85, 0.9, 0.92), 1)
  k.box("metal", V(W - 0.03, H / 2, 0.025), V(0.02, 0.1, 0.012), 0.004, STEEL, 1)
  kit.put(m, door, k)
  door:shape("door", { box = { W, H, 0.02 }, pos = { W / 2, H / 2, 0.01 } })
  door:marker("handle", { pos = { W - 0.03, H / 2, 0.04 }, role = "hold" })
elseif a.kind == "reception_bell" then
  c.turned("paint", V(0, 0, 0), { { 0, 0 }, { 0.055, 0 }, { 0.055, 0.012 }, { 0, 0.012 } }, kit.BLACK, 1, 20)
  c.turned("metal", V(0, 0.012, 0), { { 0, 0 }, { 0.05, 0 }, { 0.048, 0.02 }, { 0.03, 0.04 }, { 0.008, 0.048 }, { 0, 0.048 } }, V(0.82, 0.83, 0.85), 1, 24)
  kit.put(m, m, c)
  m:shape("bell", { box = { 0.1, 0.06, 0.1 }, pos = { 0, 0.03, 0 } })
  m:marker("ring", { pos = { 0, 0.05, 0 } })
  local button = m:node("button", { pos = { 0, 0.06, 0 }, motion = { slide = { 0, -1, 0 }, min = 0, max = 0.012 } })
  local k = kit.maker()
  k.turned("metal", V(0, 0, 0), { { 0, 0 }, { 0.004, 0 }, { 0.004, 0.012 }, { 0.012, 0.012 }, { 0.012, 0.018 }, { 0, 0.02 } }, V(0.82, 0.83, 0.85), 1, 12)
  kit.put(m, button, k)
  button:marker("button", { pos = { 0, 0.02, 0 }, role = "press" })
elseif a.kind == "teller_window" then
  local W, H, HEAD = 1.0, 0.9, 0.2
  for _, x in ipairs{ -W / 2 + 0.025, W / 2 - 0.025 } do c.box("wood", V(x, H / 2, 0), V(0.05, H, 0.08), 0.006, kit.wood("walnut"), 1) end
  c.box("wood", V(0, H - HEAD / 2, 0.01), V(W - 0.1, HEAD, 0.12), 0.006, kit.wood("walnut"), 1)               -- the box the blind rolls into
  c.box("glass", V(0, (H - HEAD + 0.06) / 2, 0), V(W - 0.1, H - HEAD - 0.06, 0.008), 0.0, V(0.85, 0.9, 0.92), 1)
  c.mesh("metal", mesh.translate(kit.facing(mesh.lathe{ profile = { { 0.05, -0.006 }, { 0.07, -0.006 }, { 0.07, 0.006 }, { 0.05, 0.006 } }, segments = 20 }), { 0, 0.5, 0 }), STEEL)   -- the grille to speak through
  kit.put(m, m, c)
  kit.far_box(m, m, "glass", { W, H, 0.02 }, { 0, H / 2, 0 }, V(0.85, 0.9, 0.92), far)
  m:shape("glass", { box = { W, H - 0.06, 0.02 }, pos = { 0, (H + 0.06) / 2, 0 } })
  m:marker("speak", { pos = { 0, 0.5, 0.02 } })
  local tray = m:node("tray", { pos = { 0, 0.0, -0.1 }, motion = { slide = { 0, 0, 1 }, min = 0, max = 0.2 } })
  local t = kit.maker()
  t.box("metal", V(0, 0.006, 0), V(0.3, 0.012, 0.2), 0.004, STEEL, 1)
  for _, x in ipairs{ -0.15, 0.15 } do t.box("metal", V(x, 0.02, 0), V(0.008, 0.04, 0.2), 0.0, STEEL, 1) end
  t.box("hard", V(0, 0.014, 0.02), V(0.16, 0.003, 0.08), 0.0, V(0.6, 0.72, 0.6), 1)                    -- what is passed across
  kit.put(m, tray, t)
  tray:marker("tray", { pos = { 0, 0.02, 0.08 }, role = "hold" })
  -- the blind: five strips, each dropping to its own place out of the box above (the projector screen's way)
  local each = (H - HEAD - 0.06) / 5
  for i = 1, 5 do
    local n = m:node("blind_" .. i, { pos = { 0, H - HEAD + 0.02, 0.02 + i * 0.0006 }, motion = { slide = { 0, -1, 0 }, min = 0, max = (i - 1) * each + 0.02 } })
    local b = kit.maker()
    b.box("paint", V(0, -each / 2, 0), V(W - 0.1, each + 0.02, 0.003), 0.0, V(0.3, 0.42, 0.34), 1)
    kit.put(m, n, b)
  end
  local bar = m:node("bar", { pos = { 0, H - HEAD + 0.02, 0.024 }, motion = { slide = { 0, -1, 0 }, min = 0, max = 5 * each + 0.02 } })
  local b = kit.maker()
  b.box("metal", V(0, -0.01, 0), V(W - 0.1, 0.02, 0.012), 0.004, BRASS, 1)
  kit.put(m, bar, b)
  bar:marker("pull", { pos = { 0, -0.01, 0.02 }, role = "hold" })
else
  c.turned("metal", V(0, 0, 0), { { 0, 0 }, { 0.18, 0 }, { 0.18, 0.02 }, { 0.03, 0.04 }, { 0.03, 1.0 }, { 0, 1.0 } }, STEEL, 1, 20)
  c.box("paint", V(0, 1.12, 0), V(0.26, 0.26, 0.16), 0.04, kit.paint.red, 3)
  c.box("hard", V(0, 1.05, 0.081), V(0.1, 0.012, 0.002), 0.0, kit.BLACK, 1)                             -- the mouth
  c.box("hard", V(0, 1.4, 0), V(0.3, 0.2, 0.05), 0.01, DARK, 1)                                         -- the number now being served
  c.box("light", V(-0.05, 1.4, 0.026), V(0.06, 0.12, 0.001), 0.0, V(1, 0.25, 0.1), 1)
  c.box("light", V(0.05, 1.4, 0.026), V(0.06, 0.12, 0.001), 0.0, V(1, 0.25, 0.1), 1)
  c.rod("metal", V(0, 1.24, 0), V(0, 1.3, 0), 0.012, STEEL, 6)
  kit.put(m, m, c)
  kit.far_box(m, m, "paint", { 0.26, 0.5, 0.16 }, { 0, 1.25, 0 }, kit.paint.red, far)
  m:shape("post", { capsule = { radius = 0.1, height = 1.2 }, pos = { 0, 0.65, 0 } })
  m:marker("number", { pos = { 0, 1.4, 0.03 }, role = "screen", size = 0.2 })
  local ticket = m:node("ticket", { pos = { 0, 1.05, 0.0 }, motion = { slide = { 0, 0, 1 }, min = 0, max = 0.14 } })
  local k = kit.maker()
  k.box("hard", V(0, 0, 0), V(0.07, 0.001, 0.12), 0.0, V(0.97, 0.97, 0.9), 1)
  kit.put(m, ticket, k)
  ticket:marker("ticket", { pos = { 0, 0, 0.06 }, role = "hold" })
end
return m
