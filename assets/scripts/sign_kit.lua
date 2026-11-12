-- sign_kit.lua -- THE SIGN KIT'S RECIPES (src/engine/procgen/world/sign_kit.h). How any road sign is drawn and built:
-- plate outlines, a face builder (shapes, strokes, lettering), the symbols signs share (arrows, the red circle and
-- slash, a signal head, a walking figure, a bicycle), and the 3-D model (the plate with its rim and back, the posts,
-- clamps, mast arm or overhead span), near or far, new or worn.
--
-- WHICH signs exist is not here: a catalogue (signs_us.lua) lists them over these recipes.
--
-- Units: a face is designed in INCHES, as the US sign manual (the MUTCD, below) and its drawings give every size, with
-- the origin at the plate's centre and y up. The face builder turns that into the engine's SignFace (metres from the
-- top-left corner, y down) and the model into metres.
--
-- Sides: the engine drives on the LEFT. Nothing here assumes a side. A sign that depends on one (a turn arrow, KEEP
-- LEFT, an exit arrow, the side a mast arm reaches over the road) is given "near" (the kerb side's turn) or "far",
-- and kit.side() turns that into the left or right the legend shows, from the catalogue's `kerb_side`.

local kit = {}
local P = mesh.profile

kit.IN = 0.0254   -- metres in an inch
kit.FT = 0.3048
local IN = kit.IN

-- ===================================================================================================================
-- SIDES
-- ===================================================================================================================

-- "near" / "far" (a turn toward the kerb, or across the traffic) -> "left" / "right" as a legend shows it, for a road
-- whose kerb is on `kerb_side`. "straight", "u", "left" and "right" pass through.
function kit.side(turn, kerb_side)
  if turn == "near" then return kerb_side end
  if turn == "far" then return kerb_side == "left" and "right" or "left" end
  return turn
end

-- the reader's right is +x on a face: "left" -1, anything else +1
local function sx(dir) return dir == "left" and -1 or 1 end
kit.sx = sx

-- ===================================================================================================================
-- OUTLINES: a plate's edge, counter-clockwise round its centre, y up, inches; corners left sharp (kit.round rounds them)
-- ===================================================================================================================

kit.shapes = {}
function kit.shapes.rect(w, h) return { { -w / 2, -h / 2 }, { w / 2, -h / 2 }, { w / 2, h / 2 }, { -w / 2, h / 2 } } end
-- a regular octagon `w` across its flats, a flat on top (STOP)
function kit.shapes.octagon(w)
  local a = w / 2
  local s = a * math.tan(math.pi / 8)
  return { { s, -a }, { a, -s }, { a, s }, { s, a }, { -s, a }, { -a, s }, { -a, -s }, { -s, -a } }
end
-- an equilateral triangle with sides `w`, point down (YIELD): `h` is its height
function kit.shapes.triangle_down(w)
  local h = w * math.sqrt(3) / 2
  return { { 0, -h / 2 }, { w / 2, h / 2 }, { -w / 2, h / 2 } }, h
end
-- a square with sides `w` stood on a corner (the warning diamond); it spans w * sqrt 2 each way
function kit.shapes.diamond(w)
  local d = w / math.sqrt(2)
  return { { 0, -d }, { d, 0 }, { 0, d }, { -d, 0 } }
end
-- a house shape, point up (the school sign): its sides run straight up `shoulder` of the height
function kit.shapes.pentagon(w, h, shoulder)
  local s = -h / 2 + h * (shoulder or 0.5)
  return { { -w / 2, -h / 2 }, { w / 2, -h / 2 }, { w / 2, s }, { 0, h / 2 }, { -w / 2, s } }
end
function kit.shapes.circle(w) return P.circle(w / 2, 48) end

-- The outline and its bounding size for a catalogue entry's shape and size (inches).
function kit.outline(shape, size)
  local w, h = size[1], size[2] or size[1]
  if shape == "octagon" then return kit.shapes.octagon(w), w, w end
  if shape == "triangle_down" then
    local ring, th = kit.shapes.triangle_down(w)
    return ring, w, th
  end
  if shape == "diamond" then return kit.shapes.diamond(w), w * math.sqrt(2), w * math.sqrt(2) end
  if shape == "pentagon" then return kit.shapes.pentagon(w, h, size.shoulder), w, h end
  if shape == "circle" then return kit.shapes.circle(w), w, w end
  return kit.shapes.rect(w, h), w, h
end

-- every corner rounded to `r` inches in `segs` steps (0: sharp)
function kit.round(ring, r, segs)
  if not r or r <= 0 or (segs or 0) < 1 then return ring end
  return P.fillet(ring, r, segs)
end

-- the outline moved inward `d` inches (a border's edges), its corners rounded to what is left of `r`
function kit.inset(ring, d, r, segs)
  local inner = P.offset(ring, -d)
  return kit.round(inner, math.max(0, (r or 0) - d), segs)
end

-- ===================================================================================================================
-- SHAPES ON A FACE (inches, plate centre, y up)
-- ===================================================================================================================

-- a circle as a ring of points round (cx, cy)
function kit.circle(cx, cy, r, sides)
  local out = {}
  local n = sides or 40
  for i = 0, n - 1 do
    local a = 2 * math.pi * i / n
    out[#out + 1] = { cx + r * math.cos(a), cy + r * math.sin(a) }
  end
  return out
end

-- a rounded rectangle centred on (cx, cy)
function kit.rounded_rect(cx, cy, w, h, r)
  local ring = P.rounded_rect(w, h, math.min(r, math.min(w, h) / 2 - 1e-6), 4)
  local out = {}
  for i, p in ipairs(ring) do out[i] = { p[1] + cx, p[2] + cy } end
  return out
end

-- A polyline as a flat band `width` wide: square ends, mitred bends (an arrow's shaft keeps its square tail).
function kit.thick(pts, width)
  local n = #pts
  local hw = width / 2
  local L, R = {}, {}
  local function dir(a, b)
    local dx, dy = b[1] - a[1], b[2] - a[2]
    local l = math.sqrt(dx * dx + dy * dy)
    if l < 1e-9 then return 0, 1 end
    return dx / l, dy / l
  end
  for i = 1, n do
    local p = pts[i]
    local nx, ny
    if i == 1 then
      local dx, dy = dir(pts[1], pts[2])
      nx, ny = -dy, dx
    elseif i == n then
      local dx, dy = dir(pts[n - 1], pts[n])
      nx, ny = -dy, dx
    else
      local ax, ay = dir(pts[i - 1], p)
      local bx, by = dir(p, pts[i + 1])
      local mx, my = -ay - by, ax + bx
      local ml = math.sqrt(mx * mx + my * my)
      if ml < 1e-9 then mx, my, ml = -ay, ax, 1 end
      mx, my = mx / ml, my / ml
      local s = 1 / math.max(0.25, mx * -ay + my * ax)   -- the mitre: the band keeps its width round the bend
      nx, ny = mx * s, my * s
    end
    L[i] = { p[1] + nx * hw, p[2] + ny * hw }
    R[i] = { p[1] - nx * hw, p[2] - ny * hw }
  end
  local ring = {}
  for i = 1, n do ring[#ring + 1] = L[i] end
  for i = n, 1, -1 do ring[#ring + 1] = R[i] end
  return ring
end

-- an arc's points round (cx, cy), radius r, from angle a0 to a1 (radians) in `n` steps
function kit.arc(cx, cy, r, a0, a1, n)
  local out = {}
  for i = 0, n do
    local a = a0 + (a1 - a0) * i / n
    out[#out + 1] = { cx + r * math.cos(a), cy + r * math.sin(a) }
  end
  return out
end

-- the points mirrored across x = x0 (a right-hand arrow made left-hand)
function kit.mirror(pts, x0)
  local out = {}
  for i, p in ipairs(pts) do out[i] = { 2 * (x0 or 0) - p[1], p[2] } end
  return out
end

-- ===================================================================================================================
-- THE FACE BUILDER: draws in inches round the plate's centre, writes the engine's SignFace (metres, top-left, y down)
-- ===================================================================================================================

local Face = {}
Face.__index = Face

-- A face `w` x `h` inches. `palette` turns colour names into "#rrggbb"; `series` the letter widths (MUTCD alphabets
-- B..F as a share of the sign font's own width).
function kit.face(w, h, palette, series)
  return setmetatable({ w = w, h = h, elems = {}, palette = palette or {}, series = series or {}, background = "none" }, Face)
end

function Face:pt(x, y) return { (x + self.w / 2) * IN, (self.h / 2 - y) * IN } end
function Face:color(c) return self.palette[c] or c end
function Face:ring(ring)
  local out = {}
  for i, p in ipairs(ring) do out[i] = self:pt(p[1], p[2]) end
  return out
end

-- a filled shape: one or more rings, even-odd (a ring inside another cuts a hole)
function Face:poly(rings, color)
  local r = {}
  for i, ring in ipairs(rings) do r[i] = self:ring(ring) end
  self.elems[#self.elems + 1] = { kind = "poly", rings = r, color = self:color(color) }
end
-- a stroke `width` inches wide along the points, round ends and bends
function Face:line(pts, width, color)
  self.elems[#self.elems + 1] = { kind = "line", rings = { self:ring(pts) }, cap = width * IN, color = self:color(color) }
end
function Face:disc(cx, cy, r, color) self:poly({ kit.circle(cx, cy, r) }, color) end

-- How wide `s` sets (inches) in capitals `cap` inches tall, in letter series `series`.
function Face:text_width(s, cap, series)
  return sign_text.width(s, cap * IN, self.series[series or "D"] or 1) / IN
end

-- Lettering: `s` centred on x (or starting / ending there: anchor), its capitals' middle at y, `cap` inches tall, in
-- letter series `series`. With `fit` (inches) a line too long is set narrower, down to the catalogue's `squeeze_floor`
-- of its series, then smaller.
function Face:text(s, o)
  local xs = self.series[o.series or "D"] or 1
  local cap = o.cap
  if o.fit then
    local w = sign_text.width(s, cap * IN, xs) / IN
    if w > o.fit then
      local floor = self.series.squeeze_floor or 0.8
      local squeeze = math.max(o.fit / w, floor)
      xs = xs * squeeze
      if w * squeeze > o.fit then cap = cap * o.fit / (w * squeeze) end
    end
  end
  local p = self:pt(o.x or 0, (o.y or 0) - cap / 2)
  self.elems[#self.elems + 1] = {
    kind = "text", x = p[1], y = p[2], text = s, cap = cap * IN, color = self:color(o.color or "black"),
    anchor = o.anchor or "middle", xscale = xs,
  }
end

-- An exit tab standing on the panel's top edge: `x` its centre, `w` x `h` inches, lettered `s` (the road-sign
-- system's own tab, drawn white-bordered guide green).
function Face:tab(s, x, w, h, cap)
  local p = self:pt(x - w / 2, self.h / 2 + h)
  self.elems[#self.elems + 1] = { kind = "tab", x = p[1], y = p[2], w = w * IN, h = h * IN, text = s, cap = cap * IN }
  self.tab_in = { x = x, w = w, h = h }
end

-- The face as the engine reads it (SignFace), and the same face drawn as NIGHT SHINE: every colour replaced by the grey
-- of how strongly that sheeting returns headlight (`sheeting[colour name]`, 0..1 of white's).
function Face:finish(sheeting, label, wear, seed)
  local shineOf = {}
  for name, hex in pairs(self.palette) do
    local v = sheeting and sheeting[name]
    if v then
      local g = math.floor(math.max(0, math.min(1, v)) * 255 + 0.5)
      shineOf[hex] = string.format("#%02x%02x%02x", g, g, g)
    end
  end
  local function grey(c) return shineOf[c] or "#000000" end
  local shine = {}
  for i, e in ipairs(self.elems) do
    local s = {}
    for k, v in pairs(e) do s[k] = v end
    if s.color then s.color = grey(s.color) end
    shine[i] = s
    if e.kind == "tab" then
      -- the tab is drawn in its own colours (white edge, green field): cover it in their shine
      local x0, y0, x1, y1 = e.x, e.y, e.x + e.w, e.y + e.h + 0.08
      shine[#shine + 1] = { kind = "poly", color = grey(self.palette.white or "#ffffff"),
                            rings = { { { x0, y0 }, { x1, y0 }, { x1, y1 }, { x0, y1 } } } }
      local m = 0.04
      shine[#shine + 1] = { kind = "poly", color = grey(self.palette.green or "#00693f"),
                            rings = { { { x0 + m, y0 + m }, { x1 - m, y0 + m }, { x1 - m, y1 }, { x0 + m, y1 } } } }
      shine[#shine + 1] = { kind = "text", x = e.x + e.w / 2, y = e.y + e.h * 0.5 + e.cap / 2, cap = e.cap, text = e.text,
                            color = grey(self.palette.white or "#ffffff"), anchor = "middle" }
    end
  end
  return {
    w = self.w * IN, h = self.h * IN, background = "none", elems = self.elems,
    shine = { background = "none", elems = shine },
    wear = wear or 0, seed = seed or 0, label = label,
  }
end

-- ===================================================================================================================
-- SYMBOLS (inches, on a face)
-- ===================================================================================================================

kit.symbols = {}

-- An arrow along `pts` (its tip the last point): a square-tailed shaft `shaft` wide and a head `head_w` wide, `head_l`
-- long.
function kit.symbols.arrow_path(f, pts, o)
  local n = #pts
  local tip, prev = pts[n], pts[n - 1]
  local dx, dy = tip[1] - prev[1], tip[2] - prev[2]
  local l = math.sqrt(dx * dx + dy * dy)
  dx, dy = dx / l, dy / l
  local bx, by = tip[1] - dx * o.head_l, tip[2] - dy * o.head_l
  local shaft = {}
  for i = 1, n - 1 do shaft[i] = pts[i] end
  shaft[n] = { bx + dx * 0.2, by + dy * 0.2 }   -- into the head a little, so no seam shows between them
  f:poly({ kit.thick(shaft, o.shaft) }, o.color)
  local hx, hy = -dy * o.head_w / 2, dx * o.head_w / 2
  f:poly({ { { bx + hx, by + hy }, { tip[1], tip[2] }, { bx - hx, by - hy } } }, o.color)
end

-- straight ahead: from (x, y0) up to (x, y1)
function kit.symbols.arrow_straight(f, o)
  kit.symbols.arrow_path(f, { { o.x or 0, o.y0 }, { o.x or 0, o.y1 } }, o)
end

-- a turn: up from (x, y0) to the bend at y, round a quarter circle of radius r toward `dir`, on to the tip `reach`
-- inches out from x
function kit.symbols.arrow_turn(f, o)
  local x = o.x or 0
  local pts = { { x, o.y0 } }
  for _, p in ipairs(kit.arc(x + o.r, o.y, o.r, math.pi, math.pi / 2, 8)) do pts[#pts + 1] = p end
  pts[#pts + 1] = { x + o.reach, o.y + o.r }
  if o.dir == "left" then pts = kit.mirror(pts, x) end
  kit.symbols.arrow_path(f, pts, o)
end

-- a U-turn: up from (x, y0) to y, over a half circle of radius r toward `dir`, back down to the tip at y_tip
function kit.symbols.arrow_uturn(f, o)
  local x = o.x or 0
  local pts = { { x, o.y0 } }
  for _, p in ipairs(kit.arc(x + o.r, o.y, o.r, math.pi, 0, 14)) do pts[#pts + 1] = p end
  pts[#pts + 1] = { x + 2 * o.r, o.y_tip }
  if o.dir == "left" then pts = kit.mirror(pts, x) end
  kit.symbols.arrow_path(f, pts, o)
end

-- the red circle and slash (a prohibition): a ring `d` across with a band `width` wide, the slash from upper left to
-- lower right
function kit.symbols.prohibit(f, o)
  local r = o.d / 2
  local cx, cy = o.x or 0, o.y or 0
  f:poly({ kit.circle(cx, cy, r, 64), kit.circle(cx, cy, r - o.width, 64) }, o.color or "red")
  local c = (r - o.width / 2) * math.cos(math.pi / 4)
  f:poly({ kit.thick({ { cx - c, cy + c }, { cx + c, cy - c } }, o.width * 0.9) }, o.color or "red")
end

-- a three-lamp signal head `h` tall, centred on (x, y)
function kit.symbols.signal(f, o)
  local h = o.h
  local w = h * 0.38
  f:poly({ kit.rounded_rect(o.x or 0, o.y or 0, w, h, w * 0.25) }, "black")
  local r = w * 0.34
  for i, c in ipairs({ "red", "yellow", "green" }) do
    f:disc(o.x or 0, (o.y or 0) + h / 3 * (2 - i), r, c)
  end
end

-- a walking figure `h` tall standing on (x, y), striding toward `dir` (the US symbol walks to the reader's left)
function kit.symbols.walker(f, o)
  local s = o.h / 20
  local x, y = o.x or 0, o.y or 0
  local d = (o.dir == "right") and 1 or -1
  local function P2(px, py) return { x + d * px * s, y + py * s } end
  local w = 1.9 * s
  f:disc(x + d * -0.2 * s, y + 18.2 * s, 1.8 * s, o.color or "black")
  f:line({ P2(0, 15.3), P2(0.6, 9.2) }, w * 1.25, o.color or "black")              -- the body
  f:line({ P2(0.2, 14.2), P2(-2.4, 11.4), P2(-3.4, 8.6) }, w, o.color or "black")  -- the forward arm
  f:line({ P2(0.3, 14.2), P2(2.6, 11.8), P2(4.0, 9.8) }, w, o.color or "black")    -- the back arm
  f:line({ P2(0.6, 9.4), P2(-2.2, 5.0), P2(-4.4, 1.0) }, w, o.color or "black")    -- the forward leg
  f:line({ P2(0.6, 9.4), P2(2.0, 4.8), P2(4.6, 1.2) }, w, o.color or "black")      -- the back leg
end

-- a bicycle `w` wide on its wheels' line at y, ridden toward `dir`, with its rider when `rider`
function kit.symbols.bicycle(f, o)
  local s = o.w / 24
  local x, y = o.x or 0, o.y or 0
  local d = (o.dir == "left") and -1 or 1
  local c = o.color or "black"
  local function P2(px, py) return { x + d * px * s, y + py * s } end
  local wr, band = 5.4 * s, 1.1 * s
  for _, cx in ipairs({ -6.4, 6.4 }) do
    local q = P2(cx, 5.4)
    f:poly({ kit.circle(q[1], q[2], wr, 40), kit.circle(q[1], q[2], wr - band, 40) }, c)
  end
  local w = 1.0 * s
  f:line({ P2(-6.4, 5.4), P2(0, 5.4), P2(-1.8, 11.4), P2(-6.4, 5.4) }, w, c)   -- the rear triangle
  f:line({ P2(0, 5.4), P2(5.0, 11.0), P2(-1.8, 11.4) }, w, c)                    -- the front triangle
  f:line({ P2(5.0, 11.0), P2(6.4, 5.4) }, w, c)                                  -- the fork
  f:line({ P2(4.6, 12.6), P2(5.0, 11.0) }, w, c)                                 -- the stem
  f:line({ P2(3.6, 12.8), P2(5.6, 12.8) }, w, c)                                 -- the bars
  f:line({ P2(-3.0, 12.2), P2(-0.6, 12.2) }, w * 1.3, c)                         -- the saddle
  if o.rider then
    f:disc(x + d * 2.4 * s, y + 20.6 * s, 1.6 * s, c)
    f:line({ P2(-1.6, 13.4), P2(1.6, 18.6) }, 1.9 * s, c)                        -- the back
    f:line({ P2(1.4, 18.0), P2(4.8, 13.4) }, 1.3 * s, c)                         -- the arm to the bars
    f:line({ P2(-1.2, 13.6), P2(2.2, 10.0), P2(0.2, 6.2) }, 1.5 * s, c)          -- the leg to the pedal
  end
end

-- KEEP LEFT / KEEP RIGHT: a dark island, and an arrow coming down past it on the `dir` side
function kit.symbols.keep_side(f, o)
  local d = sx(o.dir)
  local s = o.h / 20
  f:poly({ kit.rounded_rect(-d * 3.6 * s, -2.0 * s, 5.0 * s, 9.0 * s, 1.6 * s) }, o.color or "black")
  local pts = { { -d * 2.6 * s, 10.0 * s }, { d * 3.4 * s, 4.0 * s }, { d * 3.4 * s, -10.0 * s } }
  kit.symbols.arrow_path(f, pts, { shaft = 2.6 * s, head_w = 7.4 * s, head_l = 5.2 * s, color = o.color or "black" })
end

-- ===================================================================================================================
-- THE MODEL (metres): a plate with its face, rim and back; posts, clamps, a mast arm or an overhead span
-- ===================================================================================================================

-- A PLATE `ring` (inches, centred, y up) as two parts: the FACE (front, its texture from `slot`) and the METAL (the rim
-- round its edge and the plain back), `thick` metres deep, the front at z = 0 facing +z. Built flat in the plan
-- (mesh.fill and mesh.skirt), then stood up.
function kit.plate(ring, w_in, h_in, slot, thick, metal)
  local ringM = {}
  for i, p in ipairs(ring) do ringM[i] = { p[1] * IN, p[2] * IN } end
  local w, h = w_in * IN, h_in * IN
  local front = mesh.fill(ringM, { down = true })   -- faces down in the plan: +z once stood up
  front = mesh.uv_by(front, function(p)
    return slot.u0 + (p[1] + w / 2) / w * (slot.u1 - slot.u0), slot.v0 + (h / 2 - p[3]) / h * (slot.v1 - slot.v0)
  end)
  local back = mesh.fill(ringM, { at = { 0, thick, 0 }, color = metal })
  local rim = mesh.skirt(ringM, thick, 0, { color = metal })
  local stand = -math.pi / 2   -- the plan's +z becomes up, its -y the reader's side
  return mesh.rotate_x(front, stand), mesh.rotate_x(mesh.merge({ back, rim }), stand)
end

-- A worn plate's BEND (all its parts the same way, so it stays one closed plate): bowed by up to `bow` metres at its
-- edges, and an upper corner (on the `foldSide` side, -1 or 1) folded by `fold` radians. Normals follow the shape
-- (mesh.warp).
--
-- The plate is BOLTED to what holds it: the vertical strips within `hold` metres of the lines x = holds[k] (a post, a
-- hanger) never move, and everything bends AWAY from them, toward the reader (+z). Bending only forward keeps a worn
-- plate out of its post: no point goes behind the plate's own back, and because the bow is convex the straight chords
-- between its points stay in front as well (a bow toward the post put the flat faces between the bowed edges through
-- the post). The fold's line lies past every held strip (the plate reaches no higher than `ymax`), so the corner that
-- folds never has a post or clamp behind it.
function kit.bend(parts, w, ymax, bow, fold, foldSide, holds, hold)
  local half = w / 2
  local function free(x)   -- how far x is from the nearest held strip
    local d = math.huge
    for _, hx in ipairs(holds) do d = math.min(d, math.abs(x - hx)) end
    return math.max(0, d - hold)
  end
  local nx, ny = foldSide * 0.7071, 0.7071
  local reach = -math.huge
  for _, hx in ipairs(holds) do reach = math.max(reach, foldSide * hx + hold) end
  local foldAt = 0.7071 * (reach + ymax)   -- no point of a held strip at or below ymax is past this line
  local function f(p)
    local x, y, z = p[1], p[2], p[3]
    local t = free(x) / half
    local dz = bow * t * t
    local dist = x * nx + y * ny - foldAt
    if dist > 0 then
      dz = dz + math.sin(fold) * dist
      local pull = dist * (1 - math.cos(fold))
      x, y = x - nx * pull, y - ny * pull
    end
    return { x, y, z + dz }
  end
  local out = {}
  for i, m in ipairs(parts) do out[i] = mesh.warp(m, f) end
  return out
end

-- a box `size` (metres) centred at `at`, painted
function kit.box(size, at, color)
  return mesh.paint(mesh.translate(mesh.box(size), at), color)
end
-- a round bar from `a` to `b` along x or y (a pole, an arm), `r` metres, `sides` round
function kit.bar_y(x, y0, y1, z, r, sides, color)
  return mesh.column({ x, y0, z }, r, y1 - y0, sides, color)
end
function kit.bar_x(x0, x1, y, z, r, sides, color)
  local m = mesh.column({ 0, 0, 0 }, r, math.abs(x1 - x0), sides, color)
  m = mesh.rotate_z(m, -math.pi / 2)   -- +y along +x
  return mesh.translate(m, { math.min(x0, x1), y, z })
end

return kit
