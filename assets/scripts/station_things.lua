-- PIECES FOR A STATION OR A PUBLIC HALL, each with parts that move (#191):
--
--   flap_gate        a ticket gate: two cabinets with a paddle each that swing away to let one person through
--                    (`paddle_left`, `paddle_right`)
--   departure_board  a split-flap board: `rows` lines, each a row of drums that turn a quarter at a time
--                    (`flap_R_C`, row and column) to show a new face. Its origin is the foot of its back (hang it high)
--   baggage_scanner  an x-ray machine for bags: a belt of rollers that turn (`roller_1` ...), a tunnel with a
--                    curtain of strips that swing (`strip_1` ...), and a tray riding through (`tray`)
--   turnstile_full   a full-height turnstile in a cage: three wings of bars turning a third at a time (`rotor`)
--
-- Each stands on y = 0 at its own origin; people pass through toward -z.
local kit = require "model_kit"
local V = kit.V

local a = params{
  kind = { "flap_gate", "departure_board", "baggage_scanner", "turnstile_full" },
  rows = { 5, min = 2, max = 10, step = 1 },
}
local STEEL, DARK = V(0.72, 0.73, 0.75), kit.BLACK * 1.5
local m = model.new()
local far = m:level(50)
local c = kit.maker()

if a.kind == "flap_gate" then
  local GAP = 0.6
  for i, side in ipairs{ -1, 1 } do
    local x = side * (GAP / 2 + 0.12)
    c.box("metal", V(x, 0.5, 0), V(0.24, 1.0, 1.3), 0.03, STEEL, 2)
    c.box("hard", V(x, 1.003, 0.45), V(0.14, 0.006, 0.2), 0.0, DARK, 1)                            -- where a card is touched
    c.box("light", V(x, 1.007, 0.45), V(0.05, 0.002, 0.05), 0.0, V(0.2, 1, 0.4), 1)
    m:shape("cabinet_" .. i, { box = { 0.24, 1.0, 1.3 }, pos = { x, 0.5, 0 } })
    if side > 0 then m:marker("reader", { pos = { x, 1.01, 0.45 }, role = "press" }) end
    local name = side < 0 and "paddle_left" or "paddle_right"
    local n = m:node(name, { pos = { side * GAP / 2, 0.35, 0 }, motion = { turn = { 0, -side, 0 }, min = 0, max = 85 } })
    local k = kit.maker()
    k.box("glass", V(-side * 0.145, 0.27, 0), V(0.29, 0.54, 0.012), 0.004, V(0.3, 0.6, 0.9), 1)
    k.box("paint", V(-side * 0.145, 0.27, 0), V(0.2, 0.03, 0.014), 0.0, kit.paint.orange, 1)
    kit.put(m, n, k)
    n:shape(name, { box = { 0.29, 0.54, 0.02 }, pos = { -side * 0.145, 0.27, 0 } })
  end
  kit.put(m, m, c)
  kit.far_box(m, m, "metal", { GAP + 0.48, 1.0, 1.3 }, { 0, 0.5, 0 }, STEEL, far)
  m:marker("through", { pos = { 0, 0, 0.9 }, turn = { 0, 180, 0 }, role = "stand" })
elseif a.kind == "departure_board" then
  local ROWS, COLS, CW, CH = math.floor(a.rows + 0.5), 14, 0.11, 0.16
  local W, H = COLS * CW + 0.3, ROWS * (CH + 0.04) + 0.3
  c.box("paint", V(0, H / 2, 0.06), V(W, H, 0.12), 0.01, DARK, 1)
  c.box("light", V(0, H - 0.09, 0.121), V(W * 0.5, 0.06, 0.002), 0.0, V(1, 0.8, 0.2), 1)             -- the heading
  kit.put(m, m, c)
  kit.far_box(m, m, "paint", { W, H, 0.12 }, { 0, H / 2, 0.06 }, DARK, far)
  m:shape("board", { box = { W, H, 0.12 }, pos = { 0, H / 2, 0.06 } })
  m:marker("clatter", { pos = { 0, H / 2, 0.15 } })
  local tints = { V(0.95, 0.95, 0.9), V(0.95, 0.8, 0.2), V(0.9, 0.9, 0.85), V(0.5, 0.8, 0.95) }
  local r = CH / 2
  for row = 1, ROWS do
    for col = 1, COLS do
      if col ~= 5 and col ~= 11 then                                                               -- gaps between time, place and platform
        local x, y = (col - (COLS + 1) / 2) * CW, H - 0.2 - (row - 0.5) * (CH + 0.04)
        local n = m:node(string.format("flap_%d_%d", row, col), { pos = { x, y, 0.1 }, motion = { turn = { 1, 0, 0 } } })
        local k = kit.maker()
        for f = 0, 3 do                                                                            -- four faces, each with a different mark
          local face = mesh.merge{ mesh.paint(mesh.translate(mesh.box{ CW - 0.012, CH - 0.012, 0.002 }, { 0, 0, r }), kit.BLACK * 0.8),
                                   mesh.paint(mesh.translate(mesh.box{ (CW - 0.04) * (0.4 + 0.2 * ((row + col + f) % 4)), CH * (0.25 + 0.12 * ((col * 3 + f) % 4)), 0.001 }, { 0, 0, r + 0.0015 }),
                                              tints[(row * 2 + col + f) % 4 + 1]) }
          k.mesh("hard", mesh.rotate_x(face, f * math.pi / 2))
        end
        kit.put(m, n, k)
      end
    end
  end
elseif a.kind == "baggage_scanner" then
  local LEN, WIDE, TOP = 2.6, 0.7, 0.75
  for _, x in ipairs{ -WIDE / 2 - 0.03, WIDE / 2 + 0.03 } do
    c.box("metal", V(x, TOP - 0.04, 0), V(0.05, 0.08, LEN), 0.008, STEEL, 1)
    for _, z in ipairs{ -LEN / 2 + 0.1, LEN / 2 - 0.1 } do c.box("metal", V(x, (TOP - 0.08) / 2, z), V(0.05, TOP - 0.08, 0.05), 0.006, STEEL, 1) end
  end
  -- the tunnel: two sides and a top, open at both ends
  for _, x in ipairs{ -WIDE / 2 - 0.1, WIDE / 2 + 0.1 } do c.box("paint", V(x, 0.7, 0), V(0.14, 1.4, 1.1), 0.02, V(0.85, 0.86, 0.84), 2) end
  c.box("paint", V(0, 1.3, 0), V(WIDE + 0.06, 0.2, 1.1), 0.02, V(0.85, 0.86, 0.84), 2)
  c.box("hard", V(WIDE / 2 + 0.172, 1.0, 0.2), V(0.004, 0.2, 0.3), 0.0, DARK, 1)
  kit.put(m, m, c)
  kit.far_box(m, m, "paint", { WIDE + 0.34, 1.4, 1.1 }, { 0, 0.7, 0 }, V(0.85, 0.86, 0.84), far)
  m:shape("belt", { box = { WIDE + 0.1, TOP, LEN }, pos = { 0, TOP / 2, 0 }, friction = 0.6 })
  for i, x in ipairs{ -WIDE / 2 - 0.1, WIDE / 2 + 0.1 } do m:shape("side_" .. i, { box = { 0.14, 1.4, 1.1 }, pos = { x, 0.7, 0 } }) end
  local N = 20
  for i = 1, N do
    local n = m:node("roller_" .. i, { pos = { 0, TOP - 0.03, -LEN / 2 + (i - 0.5) * LEN / N }, motion = { turn = { -1, 0, 0 } } })
    local k = kit.maker()
    k.mesh("metal", mesh.rotate_z(mesh.lathe{ profile = { { 0, -WIDE / 2 }, { 0.028, -WIDE / 2 }, { 0.028, WIDE / 2 }, { 0, WIDE / 2 } }, segments = 8 }, math.pi / 2), STEEL)
    k.box("hard", V(0, 0.027, 0), V(WIDE - 0.1, 0.004, 0.01), 0.0, DARK, 1)                        -- a mark, so the turning shows
    kit.put(m, n, k)
  end
  for _, z in ipairs{ -0.55, 0.55 } do
    for i = 1, 7 do
      local n = m:node(string.format("strip_%d", (z < 0 and 0 or 7) + i), { pos = { -WIDE / 2 + (i - 0.5) * WIDE / 7, 1.2, z }, motion = { turn = { 1, 0, 0 }, min = -50, max = 50 } })
      local k = kit.maker()
      k.box("rubber", V(0, -0.21, 0), V(WIDE / 7 - 0.004, 0.42, 0.004), 0.0, kit.BLACK * 1.2, 1)
      kit.put(m, n, k)
    end
  end
  local tray = m:node("tray", { pos = { 0, TOP, LEN / 2 - 0.3 }, motion = { slide = { 0, 0, -1 }, min = 0, max = LEN - 0.6 } })
  local t = kit.maker()
  t.box("hard", V(0, 0.02, 0), V(0.5, 0.04, 0.4), 0.01, V(0.45, 0.47, 0.5), 1)
  t.box("fabric", V(0, 0.14, 0), V(0.36, 0.22, 0.26), 0.04, kit.fabric("navy"), 2)                 -- a bag in it
  kit.put(m, tray, t)
  m:marker("put_down", { pos = { 0, TOP, LEN / 2 - 0.3 } })
  m:marker("pick_up", { pos = { 0, TOP, -LEN / 2 + 0.3 } })
else
  local R, H = 0.75, 2.2
  c.rod("metal", V(0, 0, 0), V(0, H, 0), 0.02, STEEL, 8)
  c.box("metal", V(0, H + 0.03, 0), V(2 * R + 0.3, 0.06, 2 * R + 0.3), 0.01, STEEL, 1)
  -- the cage: bars round the half a person does not walk through, and a comb of bars the wings pass between
  for i = 0, 10 do
    local t = math.rad(-90 + i * 18)
    c.rod("metal", V(-(R + 0.08) * math.cos(t), 0, (R + 0.08) * math.sin(t)), V(-(R + 0.08) * math.cos(t), H, (R + 0.08) * math.sin(t)), 0.012, STEEL, 6)
  end
  for _, z in ipairs{ -R - 0.08, R + 0.08 } do c.rod("metal", V(R + 0.08, 0, z), V(R + 0.08, H, z), 0.02, STEEL, 8) end
  for j = 0, 9 do c.rod("metal", V(R + 0.08, 0.2 + j * 0.2, 0), V(0.1, 0.2 + j * 0.2, 0), 0.01, STEEL, 6) end
  kit.put(m, m, c)
  kit.far_box(m, m, "metal", { 2 * R, H, 2 * R }, { 0, H / 2, 0 }, STEEL, far)
  m:shape("cage", { box = { 0.1, H, 2 * R }, pos = { -R - 0.05, H / 2, 0 } })
  m:shape("comb", { box = { R, H, 0.05 }, pos = { R / 2 + 0.08, H / 2, 0 } })
  local rotor = m:node("rotor", { pos = { 0, 0, 0 }, motion = { turn = { 0, 1, 0 } } })
  local k = kit.maker()
  for w = 0, 2 do
    local t = math.rad(60 + w * 120)
    for j = 0, 9 do k.rod("metal", V(0, 0.3 + j * 0.2, 0), V((R - 0.02) * math.cos(t), 0.3 + j * 0.2, (R - 0.02) * math.sin(t)), 0.012, STEEL, 6) end
  end
  kit.put(m, rotor, k)
  rotor:marker("push", { pos = { R * 0.5 * math.cos(math.rad(60)), 1.1, R * 0.5 * math.sin(math.rad(60)) }, role = "hold" })
end
return m
