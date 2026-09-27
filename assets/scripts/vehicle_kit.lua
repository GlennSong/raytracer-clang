-- vehicle_kit.lua -- fleet v2 bodies built on the poly.* kit (ADR-0139).
--
-- A car body is ONE LOFT of cross-sections from tail to nose. Every section has the same ten-corner
-- outline -- bottom, rocker bulge, shoulder, glass base and roof edge, on each side -- rounded by
-- per-corner fillets, so sections correspond point for point and the loft is a clean quad cage. What
-- makes a sedan a sedan (or a hatch, an SUV, a pickup cab) is the PROFILE CURVES along the car: the hood
-- and deck line, the beltline, the windshield and backlight, roof width, plan-view taper, and the arches
-- over the axles. Then the kit does what a modeller would: glass assigned and pushed in behind a trim
-- reveal, lamps and grille set into the nose and tail, creases where the eye expects an edge, and
-- Catmull-Clark subdivision (level 1 for traffic, 2 for the hero car).
--
-- Axes: x right, y up, z forward (+z is the nose). The body's y = 0 is the GROUND; the city's recipes
-- shift to the body-centre origin at the end.

local kit = {}

local function lerp(a, b, t) return a + (b - a) * t end
local function clamp01(t) return math.max(0, math.min(1, t)) end
local function smooth(t) t = clamp01(t); return t * t * (3 - 2 * t) end
-- piecewise-smooth curve through keys { {z, v}, ... } (ascending z)
local function curve(keys, z)
  if z <= keys[1][1] then return keys[1][2] end
  for i = 1, #keys - 1 do
    local a, b = keys[i], keys[i + 1]
    if z <= b[1] then return lerp(a[2], b[2], smooth((z - a[1]) / (b[1] - a[1]))) end
  end
  return keys[#keys][2]
end

-- The corner layout of every section (CCW seen from +z, i.e. from the nose looking back... x right, y up):
--   1 bottom-left   2 bottom-right   3 side-right (widest)   4 shoulder-right   5 glass-base-right
--   6 roof-right    7 roof-left      8 glass-base-left       9 shoulder-left    10 side-left
local CORNERS = 10
local SEGS = 3                        -- fillet steps per corner: every corner emits SEGS + 1 points
local PER = SEGS + 1                  -- points per corner
-- which corner-to-corner run a section point belongs to (its point index is (corner-1)*PER + j)
local RUN = { bottom = { 1, 2 }, side_r = { 2, 4 }, glass_r = { 5, 6 }, roof = { 6, 7 }, glass_l = { 7, 8 }, side_l = { 8, 10 } }

-- One section at station z, from the profile `P` (all heights from the ground, metres).
local function section(P, z)
  local hw = P.half_width(z)                    -- body half-width here (plan view)
  local y0 = P.bottom(z)                        -- underside (lifted over the arches)
  local ys = lerp(y0, P.belt(z), 0.45)          -- widest point of the flank
  local belt = P.belt(z)                        -- the shoulder / window sill
  local roof = math.max(P.roof(z), belt + 0.012)
  local inCabin = (roof - belt) > 0.05
  local glassBase = inCabin and (belt + 0.02) or (belt + 0.004)
  local rw = inCabin and hw * P.roof_width(z) or hw * 0.86   -- roof edge (or the hood's crown edge)
  local gw = inCabin and hw * P.glass_width or hw * 0.90     -- glass base (tumblehome above it)
  local corners = {
    { -hw * 0.90, y0 }, { hw * 0.90, y0 },
    { hw, ys }, { hw * 0.975, belt },
    { gw, glassBase }, { rw, roof },
    { -rw, roof }, { -gw, glassBase },
    { -hw * 0.975, belt }, { -hw, ys },
  }
  local r = P.radii
  local radii = { r.bottom, r.bottom, r.side, r.shoulder, r.glass, r.roof, r.roof, r.glass, r.shoulder, r.side }
  return poly.section(corners, radii, SEGS), inCabin
end

-- Build a body from profile `P`. Returns the (unsubdivided) poly and a few facts the recipe needs.
function kit.body(P)
  local stations = {}
  local zs = P.stations
  for i = 1, #zs do stations[i] = zs[i] end
  local sections, cabin = {}, {}
  for i, z in ipairs(stations) do sections[i], cabin[i] = section(P, z) end
  local n = #sections[1]
  for i = 2, #sections do
    assert(#sections[i] == n, "section " .. i .. " has " .. #sections[i] .. " points, not " .. n)
  end
  local B = poly.loft{ sections = sections, stations = stations, cap_start = true, cap_end = true,
                       cap_crease = P.cap_crease or 0.3 }

  -- face index of the quad from ring k (1-based) point i (1-based): (k-1)*n + (i-1), 0-based
  local function face(k, i) return (k - 1) * n + (i - 1) end
  -- the quads of a run between two corners: quad i joins section points i and i+1 (1-based). `straight`
  -- takes only the flat stretch between the two corners' fillets (the glass, not the pillar edges);
  -- otherwise the fillets are included
  local function run(name, straight)
    local a, b = RUN[name][1], RUN[name][2]
    local out = {}
    local i0, i1 = straight and a * PER or (a - 1) * PER + 1, straight and (b - 1) * PER + 1 - 1 or b * PER - 1
    for i = i0, i1 do out[#out + 1] = i end
    return out
  end
  local function faces(runName, pred, straight)
    local sel = {}
    for k = 1, #stations - 1 do
      local zm = 0.5 * (stations[k] + stations[k + 1])
      if pred(zm, k) then
        for _, i in ipairs(run(runName, straight)) do
          if i < n then sel[#sel + 1] = face(k, i) end
        end
      end
    end
    return sel
  end

  local G = P.glass
  -- WINDSHIELD and BACKLIGHT: the roof run between the cowl and the A-pillar top, and between the
  -- C-pillar top and the deck
  local windshield = faces("roof", function(z) return z < G.cowl and z > G.a_top end, true)
  local backlight = faces("roof", function(z) return z > G.deck and z < G.c_top end, true)
  -- SIDE GLASS: the glass runs between the A and C pillars, less the B-pillar band
  local function sideOK(z)
    if not (z < G.a_side and z > G.c_side) then return false end
    for _, pz in ipairs(G.pillars or {}) do if math.abs(z - pz) < (G.pillar_half or 0.09) then return false end end
    return true
  end
  local sideR = faces("glass_r", sideOK, true)
  local sideL = faces("glass_l", sideOK, true)
  -- the runs' corner points belong to the pillars: drop the first and last point of each run's band
  local glass = {}
  for _, s in ipairs({ windshield, backlight, sideR, sideL }) do for _, f in ipairs(s) do glass[#glass + 1] = f end end
  B:assign(glass, { mat = "glass", group = "glass" })

  -- the underside and the arches: dark
  local under = faces("bottom", function() return true end, true)
  B:assign(under, { mat = "trim", group = "underbody" })

  -- set the glass IN: a shallow region extrude whose side walls become a dark reveal. The WINDOW SHAPE
  -- CONTROL is per window type (P.windows.windshield / side / back = { edge =, corner =, depth = }):
  --   edge    crease of the glass outline and of the body's opening (0.5 soft .. 3 crisp)
  --   corner  vertex sharpness where the outline turns (0 round, 1 slightly eased, 3+ square)
  local W = P.windows or {}
  local function setIn(sel, spec, wallMat)
    if #sel == 0 then return end
    spec = spec or {}
    local edge, corner = spec.edge or 0.8, spec.corner or 0.0
    local before = B:face_count()
    B:extrude(sel, -(spec.depth or 0.02))
    local walls = {}
    for f = before, B:face_count() - 1 do walls[#walls + 1] = f end
    B:assign(walls, { mat = wallMat })
    local opening = {}
    for _, f in ipairs(sel) do opening[#opening + 1] = f end
    for _, f in ipairs(walls) do opening[#opening + 1] = f end
    B:crease_border(sel, edge)          -- the glass's own edge
    B:crease_border(opening, edge)      -- the body's opening
    if corner > 0 then
      B:crease_corners(sel, 35, corner)
      B:crease_corners(opening, 35, corner)
    end
  end
  setIn(windshield, W.windshield, "trim")
  setIn(backlight, W.back, "trim")
  setIn(sideR, W.side, "trim")
  setIn(sideL, W.side, "trim")

  -- THE GRILLE: the nose cap, inset from its rim and set in
  local nose = B:select{ group = "cap_end" }
  if #nose > 0 and P.grille ~= false then
    local g = B:inset(nose, P.grille_margin or 0.10)
    local before = B:face_count()
    B:extrude(g, -0.03)
    local walls = {}
    for f = before, B:face_count() - 1 do walls[#walls + 1] = f end
    B:assign(g, { mat = "grille" })
    B:assign(walls, { mat = "trim" })
    B:crease_border(g, 1.5)
    B:crease_corners(g, 35, 2)
  end

  -- the recipe's own operations (a convertible's cockpit, a pickup's bed, a van's rear door)
  if P.post then P.post(B, { faces = faces, P = P }) end
  return B, { n = n, stations = stations }
end

-- ONE PARAMETRIC BODY. `o` (all metres, z from the car's centre, + toward the nose; heights from the ground):
--   L W H clear r fo ro         length, width, height, ground clearance, wheel radius, overhangs
--   axles                       optional z list (default: from the overhangs)
--   glass = { cowl, a_top, c_top, deck, a_side, c_side, pillars = {z...}, pillar_half }
--   belt  = { {t, y}, ... }     the shoulder line, t in -1 (tail) .. 1 (nose)
--   roof_drop = {front, rear}   roof below H at the windshield top / backlight top; roof_arch the crown
--   ws_exp, bl_exp              windshield / backlight curvature (1 straight, < 1 fuller)
--   roof_w = {rear, front}      roof edge as a share of the body half-width; glass_w the glass base's
--   taper = {nose, tail, len}   plan-view rounding at the ends; tuck the lift of the underside there
--   radii = {bottom, side, shoulder, glass, roof}; lamp_y, grille_y, tail_y bands; open_top (no roof)
--   post(B, ctx), extras(car)   recipe hooks: kit operations after the loft, extra parts after the build
function kit.profile(o)
  local L, W, H = o.L, o.W, o.H
  local half = L * 0.5
  local r = o.r
  local P = { name = o.name, post = o.post, extras = o.extras, color = o.color, cap_crease = o.cap_crease,
              windows = o.windows, lens = o.lens, lamps = o.lamps, grille = o.grille, grille_margin = o.grille_margin }
  P.axles = o.axles or { -(half - o.ro), half - o.fo }
  P.wheel_r = r
  P.wheel_w = o.wheel_w or 0.21
  P.glass = o.glass
  P.glass_width = o.glass_w or 0.93
  P.length, P.width, P.height = L, W, H
  -- stations: the ends, the arches, the glass keys (each bracketed so a glass edge is a sharp step)
  local zs = {}
  local function add(z) if z >= -half and z <= half then zs[#zs + 1] = z end end
  for _, d in ipairs({ 0, 0.04, 0.12, 0.25, 0.45 }) do add(-half + d); add(half - d) end
  for _, zc in ipairs(P.axles) do
    for _, d in ipairs({ -0.46, -0.34, -0.20, 0.0, 0.20, 0.34, 0.46 }) do add(zc + d * r / 0.33) end
  end
  local g = o.glass
  for _, z in ipairs({ g.cowl, g.a_top, g.c_top, g.deck, g.a_side, g.c_side }) do add(z - 0.03); add(z + 0.03) end
  for _, pz in ipairs(g.pillars or {}) do add(pz - (g.pillar_half or 0.09) - 0.02); add(pz + (g.pillar_half or 0.09) + 0.02) end
  local step = o.step or 0.35
  for z = -half + 0.45, half - 0.45, step do add(z) end
  table.sort(zs)
  local dedup = {}
  for _, z in ipairs(zs) do if #dedup == 0 or z - dedup[#dedup] > 0.03 then dedup[#dedup + 1] = z end end
  P.stations = dedup

  local tp = o.taper or { 0.16, 0.12, 0.55 }
  function P.half_width(z)
    local e = math.max(0, math.abs(z) - (half - tp[3])) / tp[3]
    local k = z > 0 and tp[1] or tp[2]
    return W * 0.5 * (1 - k * e * e) * (1 - (z > 0 and 0.35 or 0.25) * k / 0.14 * e ^ 4)
  end
  local archGap = o.arch_gap or 0.075
  local tuck = o.tuck or 0.20
  function P.bottom(z)
    local y = o.clear
    for _, zc in ipairs(P.axles) do
      local ra = r + archGap
      local d = math.abs(z - zc)
      if d < ra then y = math.max(y, r + math.sqrt(ra * ra - d * d) * 0.97) end
    end
    local e = math.max(0, math.abs(z) - (half - 0.30)) / 0.30
    return y + tuck * e * e
  end
  local beltKeys = {}
  for i, k in ipairs(o.belt) do beltKeys[i] = { k[1] * half, k[2] } end
  function P.belt(z) return curve(beltKeys, z) end
  local rd = o.roof_drop or { 0.03, 0.05 }
  local arch = o.roof_arch or 0.03
  function P.roof(z)
    if z >= g.cowl or z <= g.deck then return 0 end
    if z > g.a_top then return lerp(P.belt(g.cowl) + 0.02, H - rd[1], smooth((g.cowl - z) / (g.cowl - g.a_top)) ^ (o.ws_exp or 0.8)) end
    if o.open_top then return 0 end
    if z < g.c_top then return lerp(P.belt(g.deck) + 0.02, H - rd[2], smooth((z - g.deck) / (g.c_top - g.deck)) ^ (o.bl_exp or 0.85)) end
    local t = (z - g.c_top) / (g.a_top - g.c_top)
    return lerp(H - rd[2], H - rd[1], t) + arch * math.sin(math.pi * t) ^ 0.7
  end
  local rw = o.roof_w or { 0.70, 0.78 }
  function P.roof_width(z) return lerp(rw[1], rw[2], smooth((z - g.deck) / (g.a_top - g.deck))) end
  P.radii = o.radii or { bottom = 0.10, side = 0.45, shoulder = 0.11, glass = 0.035, roof = 0.18 }
  P.lamp_y, P.grille_y, P.tail_y = o.lamp_y or { 0.62, 0.78 }, o.grille_y or { 0.40, 0.62 }, o.tail_y or { 0.78, 0.95 }
  return P
end

-- helpers for recipes' post hooks
local function concat(...)
  local out = {}
  for _, t in ipairs({ ... }) do for _, v in ipairs(t) do out[#out + 1] = v end end
  return out
end
-- sink the top of the body between z0 and z1 into a tub (a cockpit, a bed) `depth` deep, lined with `mat`
local function tub(B, ctx, z0, z1, depth, mat)
  local sel = concat(ctx.faces("glass_r", function(z) return z > z0 and z < z1 end),
                     ctx.faces("roof", function(z) return z > z0 and z < z1 end),
                     ctx.faces("glass_l", function(z) return z > z0 and z < z1 end))
  if #sel == 0 then return end
  local before = B:face_count()
  B:extrude(sel, depth, { 0, -1, 0 })
  local walls = {}
  for f = before, B:face_count() - 1 do walls[#walls + 1] = f end
  B:assign(sel, { mat = mat })
  B:assign(walls, { mat = mat })
  B:crease_border(sel, 1.5)
end
kit.tub = tub

-- LAMP LENSES: separate rounded parts set onto the nose and tail at the profile's lamp bands (a lens
-- reads as an object; carving it out of the body's faces gave a fang). P.lamps = false for none.
function kit.lamps(P, car)
  if P.lamps == false then return end
  local half = P.length * 0.5
  local function surfaceZ(x, front)   -- the nose / tail face where the body is at least |x| wide
    local z = front and half or -half
    local dz = front and -0.01 or 0.01
    for _ = 1, 120 do
      if P.half_width(z) * 0.97 >= math.abs(x) then return z end
      z = z + dz
    end
    return z
  end
  local lens = P.lens or {}
  local hy = P.lamp_y
  local hw = P.half_width(half - 0.25)
  local hx = (lens.head_x or 0.70) * hw
  local hwid, hhei = lens.head_w or 0.34, hy[2] - hy[1]
  for _, sx in ipairs({ 1, -1 }) do
    local x = sx * hx
    local z = surfaceZ(x + sx * hwid * 0.5, true)
    car.polys[#car.polys + 1] = kit.box_part({ hwid, hhei, 0.08 }, { x, 0.5 * (hy[1] + hy[2]), z - 0.02 }, 1.2, "lamp")
  end
  local ty = P.tail_y
  if ty[1] < P.height then
    local tx = (lens.tail_x or 0.72) * P.half_width(-half + 0.25)
    local twid = lens.tail_w or 0.36
    for _, sx in ipairs({ 1, -1 }) do
      local x = sx * tx
      local z = surfaceZ(x + sx * twid * 0.5, false)
      car.polys[#car.polys + 1] = kit.box_part({ twid, ty[2] - ty[1], 0.07 }, { x, 0.5 * (ty[1] + ty[2]), z + 0.02 }, 1.2, "lamp_red")
    end
  end
end

-- a separate rounded box part (truck boxes, trailers, signs): poly parts in ground space. `bevel` is the
-- edge crease: 1 soft, 2.5 a crisp panel edge with a small radius
function kit.box_part(size, at, bevel, mat)
  local P = poly.box(size)
  P:crease_faces(P:select{ all = true }, bevel or 2.5)
  local all = P:select{ where = function() return true end }
  P:assign(all, { mat = mat or "body" })
  local S = P:subdivide(2)
  S:translate(at)
  return S
end

-- THE FLEET. Each spec is a plain table; kit.SPECS[name]() builds its profile.
kit.SPECS = {}
local S = kit.SPECS

S.sedan = function() return kit.profile{
  name = "sedan", L = 4.70, W = 1.84, H = 1.45, clear = 0.15, r = 0.33, fo = 0.95, ro = 1.05,
  glass = { cowl = 0.55, a_top = -0.05, c_top = -0.95, deck = -1.45, a_side = 0.45, c_side = -1.05, pillars = { -0.25 } },
  belt = { { -1, 0.84 }, { -0.87, 0.95 }, { -0.62, 0.98 }, { -0.1, 0.95 }, { 0.23, 0.92 }, { 0.83, 0.84 }, { 1, 0.66 } },
  windows = { side = { edge = 1.0, corner = 0.8 }, windshield = { edge = 1.0, corner = 0.6 }, back = { edge = 1.0, corner = 0.6 } },
  color = { 0.62, 0.06, 0.07 } } end

S.taxi = function()
  local P = S.sedan()
  P.name = "taxi"
  P.color = { 0.95, 0.72, 0.05 }
  P.extras = function(car)
    car.polys[#car.polys + 1] = kit.box_part({ 0.62, 0.16, 0.26 }, { 0, car.height + 0.07, -0.35 }, 1.0, "sign")
  end
  return P
end

S.hatchback = function() return kit.profile{
  name = "hatchback", L = 4.05, W = 1.78, H = 1.48, clear = 0.15, r = 0.31, fo = 0.82, ro = 0.62,
  glass = { cowl = 0.62, a_top = 0.05, c_top = -1.55, deck = -1.98, a_side = 0.52, c_side = -1.45, pillars = { -0.35 } },
  belt = { { -1, 0.82 }, { -0.9, 0.97 }, { -0.5, 0.97 }, { 0.3, 0.93 }, { 0.8, 0.84 }, { 1, 0.64 } },
  roof_drop = { 0.03, 0.06 }, bl_exp = 0.6, roof_w = { 0.74, 0.78 },
  windows = { side = { edge = 1.2, corner = 1.0 }, windshield = { edge = 1.0, corner = 0.6 }, back = { edge = 1.2, corner = 1.0 } },
  color = { 0.10, 0.36, 0.68 } } end

S.suv = function() return kit.profile{
  name = "suv", L = 4.75, W = 1.92, H = 1.78, clear = 0.21, r = 0.37, fo = 0.92, ro = 0.95,
  glass = { cowl = 0.72, a_top = 0.05, c_top = -2.05, deck = -2.33, a_side = 0.62, c_side = -2.0, pillars = { -0.3, -1.35 } },
  belt = { { -1, 1.02 }, { -0.9, 1.12 }, { -0.4, 1.13 }, { 0.3, 1.10 }, { 0.8, 1.02 }, { 1, 0.86 } },
  roof_drop = { 0.03, 0.04 }, roof_arch = 0.015, bl_exp = 0.5, roof_w = { 0.80, 0.82 },
  radii = { bottom = 0.10, side = 0.35, shoulder = 0.10, glass = 0.035, roof = 0.14 },
  lamp_y = { 0.80, 0.95 }, grille_y = { 0.52, 0.80 }, tail_y = { 0.98, 1.12 },
  windows = { side = { edge = 1.5, corner = 1.5 }, windshield = { edge = 1.2, corner = 1.0 }, back = { edge = 1.5, corner = 1.5 } },
  color = { 0.18, 0.20, 0.23 } } end

S.jeep = function() return kit.profile{
  name = "jeep", L = 4.25, W = 1.90, H = 1.86, clear = 0.27, r = 0.40, fo = 0.72, ro = 0.72,
  glass = { cowl = 0.55, a_top = 0.38, c_top = -1.95, deck = -2.10, a_side = 0.36, c_side = -1.85, pillars = { -0.45 } },
  belt = { { -1, 1.15 }, { -0.92, 1.18 }, { 0.2, 1.18 }, { 0.9, 1.12 }, { 1, 1.00 } },
  roof_drop = { 0.02, 0.02 }, roof_arch = 0.0, ws_exp = 1.0, bl_exp = 1.0, roof_w = { 0.86, 0.86 }, glass_w = 0.95,
  taper = { 0.05, 0.03, 0.3 }, tuck = 0.10, arch_gap = 0.09,
  radii = { bottom = 0.05, side = 0.08, shoulder = 0.04, glass = 0.02, roof = 0.06 },
  lamp_y = { 0.95, 1.10 }, grille_y = { 0.62, 1.02 }, tail_y = { 1.0, 1.15 },
  cap_crease = 1.0, color = { 0.24, 0.30, 0.18 },
  windows = { side = { edge = 2.5, corner = 3 }, windshield = { edge = 2.5, corner = 3 }, back = { edge = 2.5, corner = 3 } },
  extras = function(car)   -- the spare wheel on the tailgate
    car.spare = { 0, 1.05, -(4.25 * 0.5) - 0.12 }
  end } end

S.convertible = function() return kit.profile{
  name = "convertible", L = 4.45, W = 1.82, H = 1.30, clear = 0.13, r = 0.33, fo = 0.88, ro = 0.95,
  glass = { cowl = 0.45, a_top = 0.10, c_top = -1.0, deck = -1.30, a_side = 0.0, c_side = 0.0, pillars = {} },
  belt = { { -1, 0.80 }, { -0.85, 0.88 }, { -0.4, 0.90 }, { 0.2, 0.87 }, { 0.8, 0.80 }, { 1, 0.62 } },
  open_top = true, roof_drop = { 0.05, 0.1 },
  color = { 0.88, 0.88, 0.86 },
  post = function(B, ctx) kit.tub(B, ctx, -1.25, 0.05, 0.42, "interior") end } end

S.pickup = function() return kit.profile{
  name = "pickup", L = 5.60, W = 2.00, H = 1.90, clear = 0.24, r = 0.40, fo = 0.95, ro = 1.10,
  glass = { cowl = 1.05, a_top = 0.50, c_top = -0.55, deck = -0.66, a_side = 0.95, c_side = -0.50, pillars = { 0.0 } },
  belt = { { -1, 1.12 }, { -0.95, 1.14 }, { -0.2, 1.14 }, { 0.5, 1.10 }, { 0.9, 1.04 }, { 1, 0.90 } },
  roof_drop = { 0.02, 0.02 }, roof_arch = 0.01, bl_exp = 1.0, roof_w = { 0.84, 0.82 },
  taper = { 0.10, 0.03, 0.45 }, tuck = 0.12,
  radii = { bottom = 0.08, side = 0.25, shoulder = 0.07, glass = 0.03, roof = 0.10 },
  lamp_y = { 0.88, 1.02 }, grille_y = { 0.55, 0.88 }, tail_y = { 0.85, 1.05 },
  color = { 0.14, 0.30, 0.20 },
  windows = { side = { edge = 1.5, corner = 1.5 }, windshield = { edge = 1.2, corner = 1.0 }, back = { edge = 2, corner = 2 } },
  post = function(B, ctx) kit.tub(B, ctx, -2.70, -0.74, 0.48, "bed") end } end

S.step_van = function() return kit.profile{
  name = "step_van", L = 6.30, W = 2.30, H = 2.85, clear = 0.30, r = 0.42, fo = 0.85, ro = 1.35,
  glass = { cowl = 2.70, a_top = 2.45, c_top = -3.12, deck = -3.14, a_side = 2.45, c_side = 1.55, pillars = {} },
  belt = { { -1, 1.30 }, { -0.97, 1.40 }, { 0.6, 1.40 }, { 0.85, 1.38 }, { 1, 1.25 } },
  roof_drop = { 0.02, 0.02 }, roof_arch = 0.0, ws_exp = 1.0, bl_exp = 1.0, roof_w = { 0.95, 0.95 }, glass_w = 0.985,
  taper = { 0.04, 0.01, 0.25 }, tuck = 0.05, arch_gap = 0.06,
  radii = { bottom = 0.04, side = 0.05, shoulder = 0.03, glass = 0.02, roof = 0.08 },
  lamp_y = { 1.02, 1.18 }, grille_y = { 0.55, 1.0 }, tail_y = { 0.9, 1.2 },
  cap_crease = 1.5, color = { 0.36, 0.22, 0.10 },
  windows = { side = { edge = 2.5, corner = 3 }, windshield = { edge = 2.5, corner = 2.5 }, back = { edge = 2.5, corner = 3 } } } end

-- the cab of a truck whose body is a separate part (small box truck, semi tractor)
local function truck_cab(o)
  return kit.profile{
    name = o.name, L = o.L, W = o.W, H = o.H, clear = o.clear, r = o.r, fo = o.fo, ro = o.ro, axles = o.axles,
    glass = o.glass, belt = o.belt, roof_drop = { 0.03, 0.03 }, roof_arch = 0.01, ws_exp = 0.9, bl_exp = 1.0,
    roof_w = { 0.90, 0.90 }, glass_w = 0.97, taper = { 0.10, 0.02, 0.35 }, tuck = 0.06, arch_gap = 0.08,
    radii = { bottom = 0.05, side = 0.12, shoulder = 0.06, glass = 0.03, roof = 0.12 },
    lamp_y = o.lamp_y, grille_y = o.grille_y, tail_y = { 9, 9 }, cap_crease = 1.0, color = o.color,
    windows = { side = { edge = 2.5, corner = 3 }, windshield = { edge = 2, corner = 2.5 }, back = { edge = 2.5, corner = 3 } },
    extras = o.extras }
end

S.small_truck = function()
  local boxL, boxH = 4.2, 2.3
  return truck_cab{
    name = "small_truck", L = 2.3, W = 2.10, H = 2.55, clear = 0.35, r = 0.42, fo = 0.55, ro = 0.0,
    axles = { -3.7, 0.60 },
    glass = { cowl = 1.10, a_top = 0.72, c_top = -1.05, deck = -1.12, a_side = 0.70, c_side = -0.30, pillars = {} },
    belt = { { -1, 1.45 }, { 0.3, 1.45 }, { 0.8, 1.35 }, { 1, 1.15 } },
    lamp_y = { 0.75, 0.92 }, grille_y = { 0.50, 1.10 }, color = { 0.90, 0.90, 0.88 },
    extras = function(car)
      car.polys[#car.polys + 1] = kit.box_part({ 2.25, boxH, boxL }, { 0, 0.95 + boxH * 0.5, -1.15 - boxL * 0.5 - 0.1 }, 1.0, "box")
      car.polys[#car.polys + 1] = kit.box_part({ 1.0, 0.25, boxL + 1.2 }, { 0, 0.72, -1.15 - boxL * 0.5 + 0.4 }, 1.0, "trim")
    end }
end

S.semi = function()
  local trL, trW, trH = 13.6, 2.55, 2.75
  return truck_cab{
    name = "semi", L = 6.2, W = 2.50, H = 3.95, clear = 0.40, r = 0.52, fo = 1.2, ro = 0.0,
    axles = { -2.3, -1.0, 2.0 },
    glass = { cowl = 0.80, a_top = 0.35, c_top = -2.9, deck = -3.08, a_side = 0.30, c_side = -0.40, pillars = {} },
    belt = { { -1, 1.95 }, { -0.1, 1.95 }, { 0.3, 1.90 }, { 0.45, 1.70 }, { 0.9, 1.55 }, { 1, 1.30 } },
    lamp_y = { 0.95, 1.12 }, grille_y = { 0.60, 1.60 }, color = { 0.55, 0.06, 0.06 },
    extras = function(car)
      -- the trailer, hitched behind (drawn straight for now; articulation is the sim's, phase 5)
      local zt = -3.1 - 0.9 - trL * 0.5
      car.polys[#car.polys + 1] = kit.box_part({ trW, trH, trL }, { 0, 1.25 + trH * 0.5, zt }, 1.0, "trailer")
      car.trailer_axles = { zt - trL * 0.5 + 1.6, zt - trL * 0.5 + 2.9 }
    end }
end

-- The whole car at `level` subdivisions, as { parts = { [mat] = Mesh }, wheels = {...}, size = {W,H,L} }
-- in body-centre space (origin at the middle of the bounding box, the ground at y = -H/2).
function kit.build(P, level, opts)
  opts = opts or {}
  local B = kit.body(P)
  local S = B:subdivide(level or 1)
  local lo, hi = S:bounds()
  -- extras (boxes, trailer, sign) join before the centring, in ground space
  local car = { polys = {}, height = hi[2] }
  kit.lamps(P, car)
  if P.extras then P.extras(car) end
  for _, x in ipairs(car.polys) do S:append(x) end
  lo, hi = S:bounds()
  local cy = 0.5 * hi[2]                 -- ground (0) to the top
  local cz = 0.5 * (lo[3] + hi[3])
  S:translate{ 0, -cy, -cz }
  local parts = S:to_parts{ autosmooth = 40 }
  -- wheels: round, at the axles, just inside the body sides
  local r = P.wheel_r
  local ww = P.wheel_w
  local wheels = {}
  local axles = {}
  for _, zc in ipairs(P.axles) do axles[#axles + 1] = zc end
  for _, zc in ipairs(car.trailer_axles or {}) do axles[#axles + 1] = zc end
  for _, zc in ipairs(axles) do
    local hw = math.min(P.half_width(math.max(-P.length * 0.5, math.min(P.length * 0.5, zc))), P.width * 0.5)
    for _, sx in ipairs({ 1, -1 }) do
      wheels[#wheels + 1] = { pos = { sx * (hw - ww * 0.5 - 0.035), r - cy, zc - cz }, radius = r, width = ww,
                              front = zc == P.axles[#P.axles] }
    end
  end
  local spare = car.spare and { car.spare[1], car.spare[2] - cy, car.spare[3] - cz } or nil
  return { parts = parts, wheels = wheels, size = { hi[1] - lo[1], hi[2], hi[3] - lo[3] }, lift = cy,
           length = hi[3] - lo[3], spare = spare, color = P.color }
end

-- One wheel as poly parts: a rounded tyre (a lofted ring, subdivided), a dished rim and a hub, facing +x
-- (its axle on x). { tyre = Mesh, rim = Mesh } in wheel-centre space.
function kit.wheel(r, width, level)
  local w2 = width * 0.5
  local rr = r * 0.66                                   -- rim radius
  -- the tyre's section in (radial, lateral), swept around the axle by lofting rings at angles: build it
  -- as a loft along x of circular sections instead -- a torus-like tube is simplest as rings at x stations
  local function ring(rad, n)
    local pts = {}
    for i = 0, n - 1 do
      local a = 2 * math.pi * i / n
      pts[#pts + 1] = { rad * math.cos(a), rad * math.sin(a) }
    end
    return pts
  end
  local N = 24
  -- tyre: outer tread band with rounded shoulders (rings along the axle), capped by the sidewalls
  local T = poly.loft{ sections = { ring(rr, N), ring(r * 0.93, N), ring(r, N), ring(r, N), ring(r * 0.93, N), ring(rr, N) },
                       stations = { -w2, -w2 * 0.92, -w2 * 0.55, w2 * 0.55, w2 * 0.92, w2 }, cap_start = false, cap_end = false }
  local Tm = T:subdivide(level or 1)
  -- the loft ran along z: turn it onto the x axis
  Tm:map(function(x, y, z) return z, y, -x end)
  local rim = poly.loft{ sections = { ring(rr * 1.0, N), ring(rr * 0.95, N), ring(rr * 0.35, N), ring(rr * 0.2, N) },
                         stations = { w2 * 0.60, w2 * 0.70, w2 * 0.62, w2 * 0.75 }, cap_start = true, cap_end = true,
                         cap_crease = 1 }
  local Rm = rim:subdivide(level or 1)
  Rm:map(function(x, y, z) return z, y, -x end)
  local tp = Tm:to_parts{ autosmooth = 50 }
  local rp = Rm:to_parts{ autosmooth = 50 }
  return { tyre = tp.body, rim = rp.body }
end

return kit
