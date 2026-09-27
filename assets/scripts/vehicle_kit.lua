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
-- which corner-to-corner run a face belongs to
local RUN = { bottom = { 1, 2 }, side_r = { 2, 4 }, glass_r = { 5, 6 }, roof = { 6, 7 }, glass_l = { 7, 8 }, side_l = { 8, 10 } }
-- FILLET STEPS BY RADIUS: a sharp corner is one point, a small radius two, a big one four. A tiny radius
-- stepped four times left millimetre strips down the whole body (the fleet test's slivers). The radii are
-- the same at every station, so every section still has the same count.
local function segsFor(r) if r <= 0.025 then return 0 elseif r <= 0.07 then return 1 else return 3 end end

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
  local segs = {}
  for i, rr in ipairs(radii) do segs[i] = segsFor(rr) end
  return poly.section(corners, radii, segs), inCabin
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
  -- where each corner's points start (1-based), from the per-corner fillet counts
  local rr = P.radii
  local radiiList = { rr.bottom, rr.bottom, rr.side, rr.shoulder, rr.glass, rr.roof, rr.roof, rr.glass, rr.shoulder, rr.side }
  local first, last = {}, {}
  local at = 1
  for c = 1, CORNERS do
    first[c] = at
    at = at + segsFor(radiiList[c]) + 1
    last[c] = at - 1
  end
  local function run(name, straight)
    local a, b = RUN[name][1], RUN[name][2]
    local out = {}
    local i0, i1 = straight and last[a] or first[a], straight and first[b] - 1 or last[b] - 1
    for i = i0, i1 do out[#out + 1] = i end
    return out
  end
  -- `pred(zm, k, z0, z1)`: zm the row's middle, z0/z1 its two stations
  local function faces(runName, pred, straight)
    local sel = {}
    for k = 1, #stations - 1 do
      local zm = 0.5 * (stations[k] + stations[k + 1])
      if pred(zm, k, stations[k], stations[k + 1]) then
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
  -- a glass row counts only when BOTH its stations lie inside the window's span: a row straddling the
  -- cowl has one end where the greenhouse has collapsed to nothing, and insetting it folds it over
  local function inside(z0, z1, lo, hi) return z0 >= lo - 1e-6 and z1 <= hi + 1e-6 end
  local windshield = faces("roof", function(_, _, z0, z1) return inside(z0, z1, G.a_top, G.cowl) end, true)
  local backlight = faces("roof", function(_, _, z0, z1) return inside(z0, z1, G.deck, G.c_top) end, true)
  -- SIDE GLASS: the glass runs between the A and C pillars, less the B-pillar band
  local function sideOK(z, _, z0, z1)
    if not inside(z0, z1, G.c_side, G.a_side) then return false end
    for _, pz in ipairs(G.pillars or {}) do if math.abs(z - pz) < (G.pillar_half or 0.09) then return false end end
    return true
  end
  local sideR = faces("glass_r", sideOK, true)
  local sideL = faces("glass_l", sideOK, true)
  -- each window its own group (so it can be found again after subdivision) and the glass material; its
  -- SHAPE is set here on the cage -- the outline creased and its corners as square as the spec asks --
  -- and its DETAIL (gasket, reveal) after subdivision, in kit.detail. The WINDOW SHAPE CONTROL is per
  -- window type (P.windows.windshield / side / back = { edge =, corner =, gasket =, depth = }):
  --   edge    crease of the window outline (1 soft .. 3 crisp)
  --   corner  vertex sharpness where the outline turns (0 round, 1 slightly eased, 3+ square)
  local W = P.windows or {}
  local function window(sel, group, spec)
    if #sel == 0 then return end
    spec = spec or {}
    B:assign(sel, { mat = "glass", group = group })
    B:crease_border(sel, spec.edge or 2.0)
    if (spec.corner or 0) > 0 then B:crease_corners(sel, 35, spec.corner) end
  end
  if not P.open_top then   -- an open car's windshield is a separate frame; it has no roof glass
    window(windshield, "win_front", W.windshield)
    window(backlight, "win_back", W.back)
  end
  window(sideR, "win_side_r", W.side)
  window(sideL, "win_side_l", W.side)

  -- A REAR WINDOW in the tail (a boxy vehicle's back glass is on its vertical tail, not the roof run):
  -- the tail cap's ladder rungs within P.rear_window = { y0, y1 }
  if P.rear_window then
    local rw = P.rear_window
    local rear = B:select{ group = "cap_start", where = function(c) return c[2] > rw[1] and c[2] < rw[2] end }
    window(rear, "win_rear", W.back)
  end

  -- the underside and the arches: dark
  local under = faces("bottom", function() return true end, true)
  B:assign(under, { mat = "trim", group = "underbody" })

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
              windows = o.windows, lens = o.lens, lamps = o.lamps, grille = o.grille, grille_margin = o.grille_margin,
              rear_window = o.rear_window, level = o.level, grille_w = o.grille_w, open_top = o.open_top }
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
  -- ARCHES: stations evenly round each arch's semicircle (every 15 degrees), so a faceted body's wheel
  -- arch reads as a curve rather than a six-sided box; plus one either side where the arch meets the sill
  local ra = r + (o.arch_gap or 0.075)
  for _, zc in ipairs(P.axles) do
    for deg = -90, 90, 15 do add(zc + ra * math.sin(math.rad(deg))) end
    add(zc - ra - 0.08); add(zc + ra + 0.08)
  end
  local g = o.glass
  for _, z in ipairs({ g.cowl, g.a_top, g.c_top, g.deck, g.a_side, g.c_side }) do add(z - 0.03); add(z + 0.03) end
  for _, pz in ipairs(g.pillars or {}) do add(pz - (g.pillar_half or 0.09) - 0.02); add(pz + (g.pillar_half or 0.09) + 0.02) end
  local step = o.step or 0.35
  for z = -half + 0.45, half - 0.45, step do add(z) end
  table.sort(zs)
  local dedup = {}
  for _, z in ipairs(zs) do if #dedup == 0 or z - dedup[#dedup] > (o.min_station or 0.045) then dedup[#dedup + 1] = z end end
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
    if o.open_top then return 0 end   -- a convertible's windshield is its own part (a frame and glass)
    if o.roof_keys and z <= g.a_top then   -- an explicit roof line behind the windshield (a truck's sleeper)
      local keys = {}
      for i, k in ipairs(o.roof_keys) do keys[i] = { k[1], k[2] } end
      return curve(keys, z)
    end
    if z > g.a_top then return lerp(P.belt(g.cowl) + 0.02, H - rd[1], smooth((g.cowl - z) / (g.cowl - g.a_top)) ^ (o.ws_exp or 0.8)) end
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
local function concat(...)   -- a union: each face once
  local out, seen = {}, {}
  for _, t in ipairs({ ... }) do for _, v in ipairs(t) do if not seen[v] then seen[v] = true; out[#out + 1] = v end end end
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
  -- THE GRILLE: a separate panel on the flat nose cap, sized from the grille band and the nose, with
  -- horizontal bars (a recess into the cap traced the cap's merged rim and sawed its edge)
  if P.grille ~= false then
    local gy = P.grille_y
    local gw = (P.grille_w or 0.62) * P.half_width(half)
    local gh = gy[2] - gy[1]
    local cy = 0.5 * (gy[1] + gy[2])
    car.polys[#car.polys + 1] = kit.box_part({ gw * 2, gh, 0.04 }, { 0, cy, half - 0.005 }, 2.5, "grille")
    local bars = math.max(2, math.floor(gh / 0.06))
    for i = 1, bars do
      local y = gy[1] + gh * (i - 0.5) / bars
      car.polys[#car.polys + 1] = kit.box_part({ gw * 2 - 0.04, 0.012, 0.03 }, { 0, y, half + 0.012 }, 2.5, "chrome")
    end
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
function kit.box_part(size, at, bevel, mat, level)
  local P = poly.box(size)
  P:crease_faces(P:select{ all = true }, bevel or 2.5)
  local all = P:select{ where = function() return true end }
  P:assign(all, { mat = mat or "body" })
  local S = (level or 0) > 0 and P:subdivide(level) or P   -- faceted parts are plain boxes
  S:translate(at)
  return S
end

-- a CYLINDER part: `sides`-gon rings along its axis ("x", "y" or "z"), capped, centred at `at`; the
-- polygon is symmetric so its caps close as clean quad ladders
function kit.cylinder_part(radius, length, sides, at, axis, mat)
  local n = sides or 12
  local ring = {}
  for i = 0, n - 1 do
    local a = -math.pi * 0.5 + (i + 0.5) * 2 * math.pi / n
    ring[#ring + 1] = { radius * math.cos(a), radius * math.sin(a) }
  end
  local C = poly.loft{ sections = { ring, ring }, stations = { -length * 0.5, length * 0.5 }, cap_start = true, cap_end = true,
                       cap_crease = 3 }
  C:assign(C:select{}, { mat = mat or "trim" })
  if axis == "x" then C:map(function(x, y, z) return z, y, -x end)
  elseif axis == "y" then C:map(function(x, y, z) return x, z, -y end) end
  C:translate(at)
  return C
end

-- a box part tilted `rake` degrees about x (top leaning back toward -z), centred at `at`
function kit.tilted_box(size, at, rake, mat)
  local P = poly.box(size)
  P:crease_faces(P:select{}, 3)
  P:assign(P:select{}, { mat = mat or "body" })
  local a = math.rad(rake or 0)
  local c, sn = math.cos(a), math.sin(a)
  P:map(function(x, y, z) return x, y * c - z * sn, y * sn + z * c end)
  P:translate(at)
  return P
end

-- A CONVERTIBLE'S WINDSHIELD: a raked frame (two A-posts and a header rail) round a glass pane, standing
-- on the cowl. `w` wide, `h` tall along its rake, at station z (its foot), belt height y.
function kit.windshield_frame(car, w, h, z, y, rake)
  local a = math.rad(rake)
  local up, back = math.cos(a), math.sin(a)          -- the frame's own up, in (y, -z)
  local function at(u)                                -- a point u along the frame's height, centred
    return { 0, y + u * up, z - u * back }
  end
  local mid = at(h * 0.5)
  car.polys[#car.polys + 1] = kit.tilted_box({ w - 0.08, h - 0.06, 0.012 }, mid, -rake, "glass")
  local post = 0.045
  for _, sx in ipairs({ 1, -1 }) do
    car.polys[#car.polys + 1] = kit.tilted_box({ post, h, 0.05 }, { sx * (w * 0.5 - post * 0.5), mid[2], mid[3] }, -rake, "trim")
  end
  local top = at(h - 0.02)
  car.polys[#car.polys + 1] = kit.tilted_box({ w, 0.045, 0.05 }, top, -rake, "trim")
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
  -- the roof runs flat to the tail (c_top / deck past it): a boxy tail is a tall panel with the rear
  -- window in it (rear_window), not a sloped backlight
  glass = { cowl = 0.55, a_top = 0.38, c_top = -2.40, deck = -2.50, a_side = 0.36, c_side = -1.85, pillars = { -0.45 } },
  belt = { { -1, 1.15 }, { -0.92, 1.18 }, { 0.2, 1.18 }, { 0.9, 1.12 }, { 1, 1.00 } },
  roof_drop = { 0.02, 0.02 }, roof_arch = 0.0, ws_exp = 1.0, bl_exp = 1.0, roof_w = { 0.86, 0.86 }, glass_w = 0.95,
  taper = { 0.05, 0.03, 0.3 }, tuck = 0.10, arch_gap = 0.09,
  radii = { bottom = 0.05, side = 0.08, shoulder = 0.04, glass = 0.02, roof = 0.06 },
  lamp_y = { 0.95, 1.10 }, grille_y = { 0.62, 1.02 }, tail_y = { 1.0, 1.15 },
  cap_crease = 1.0, color = { 0.24, 0.30, 0.18 }, rear_window = { 1.25, 1.78 },
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
  post = function(B, ctx) kit.tub(B, ctx, -1.25, 0.05, 0.42, "interior") end,
  extras = function(car) kit.windshield_frame(car, 1.50, 0.46, 0.30, 0.87, 58) end } end

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
  cap_crease = 1.5, color = { 0.36, 0.22, 0.10 }, grille = false,
  windows = { side = { edge = 2.5, corner = 3 }, windshield = { edge = 2.5, corner = 2.5 }, back = { edge = 2.5, corner = 3 } },
  extras = function(car)   -- a flat van's grille is a panel on its nose, not a recess
    car.polys[#car.polys + 1] = kit.box_part({ 1.30, 0.42, 0.05 }, { 0, 0.78, 3.15 + 0.01 }, 2.5, "grille")
  end } end

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

-- THE SEMI (Class 8 conventional sleeper; references: ~4.1 m tall, 2.6 m wide, a 53 ft / 16.15 m trailer,
-- fifth wheel ~1.2 m high, tandem drive axles on duals, fuel tanks under the cab, stacks behind it).
-- The tractor body is a loft from the front bumper (z = +2.2) to the back of the sleeper (z = -2.2); the
-- chassis runs on behind it to the drive axles and the fifth wheel. `trailer` = "box" | "tanker".
local function semi(trailer)
  local L, W = 4.4, 2.50
  local r = 0.52
  local zF = 1.05                        -- front axle (body space)
  local zD1, zD2 = -3.35, -4.65          -- tandem drive axles
  local frameEnd = -5.35
  local fifthZ = (zD1 + zD2) * 0.5 + 0.3
  local P = kit.profile{
    name = "semi", L = L, W = W, H = 3.95, clear = 0.45, r = r, fo = 2.2 - zF, ro = 0.0, axles = { zF },
    glass = { cowl = 0.05, a_top = -0.40, c_top = -3.0, deck = -3.1, a_side = -0.45, c_side = -1.25, pillars = {} },
    belt = { { -1, 1.95 }, { -0.1, 1.95 }, { 0.02, 1.92 }, { 0.3, 1.78 }, { 0.9, 1.62 }, { 1, 1.40 } },
    roof_keys = { { -2.2, 3.90 }, { -1.3, 3.92 }, { -0.95, 3.35 }, { -0.60, 2.95 }, { -0.40, 2.92 } },
    roof_drop = { 0.02, 0.02 }, ws_exp = 1.0, roof_w = { 0.92, 0.88 }, glass_w = 0.97,
    taper = { 0.12, 0.02, 0.6 }, tuck = 0.05, arch_gap = 0.10,
    radii = { bottom = 0.03, side = 0.10, shoulder = 0.06, glass = 0.03, roof = 0.12 },
    lamp_y = { 1.05, 1.22 }, grille_y = { 0.75, 1.62 }, grille_w = 0.72, tail_y = { 9, 9 },
    cap_crease = 2.0, color = { 0.55, 0.06, 0.06 },
    windows = { side = { edge = 3, corner = 3 }, windshield = { edge = 3, corner = 2.5 }, back = { edge = 3, corner = 3 } },
    extras = function(car)
      local add = function(x) car.polys[#car.polys + 1] = x end
      -- CHASSIS: two frame rails from under the cab to the tail, a rear crossmember and bumper
      for _, sx in ipairs({ 1, -1 }) do
        add(kit.box_part({ 0.10, 0.28, 2.0 - frameEnd }, { sx * 0.45, 0.95, (2.0 + frameEnd) * 0.5 }, 3, "chassis"))
      end
      add(kit.box_part({ 1.10, 0.20, 0.12 }, { 0, 0.90, frameEnd + 0.06 }, 3, "chassis"))
      -- FIFTH WHEEL: a tilted plate over the drive axles, on its mounting
      add(kit.box_part({ 0.95, 0.10, 0.95 }, { 0, 1.18, fifthZ }, 3, "chassis"))
      add(kit.box_part({ 1.10, 0.12, 0.40 }, { 0, 1.08, fifthZ }, 3, "chassis"))
      -- DECK PLATE behind the sleeper and the steps' tanks: two cylindrical fuel tanks under the doors
      add(kit.box_part({ 1.40, 0.04, 0.55 }, { 0, 1.12, -2.5 }, 3, "chrome"))
      for _, sx in ipairs({ 1, -1 }) do
        add(kit.cylinder_part(0.33, 1.35, 12, { sx * 1.00, 0.72, -1.55 }, "z", "chrome"))
        add(kit.box_part({ 0.26, 0.05, 0.40 }, { sx * 1.18, 0.62, -0.62 }, 3, "chassis"))   -- a cab step
        -- EXHAUST STACKS: up behind the cab's corners, past the roof
        add(kit.cylinder_part(0.075, 2.9, 10, { sx * 1.02, 2.70, -2.30 }, "y", "chrome"))
        -- battery / tool box behind the tank
        add(kit.box_part({ 0.45, 0.45, 0.60 }, { sx * 0.95, 0.80, -2.60 }, 3, "chassis"))
        -- MUDFLAPS behind the rear drive axle, and quarter fenders over the drive wheels
        add(kit.box_part({ 0.62, 0.62, 0.02 }, { sx * 1.02, 0.55, zD2 - 0.62 }, 3, "trim"))
        add(kit.box_part({ 0.66, 0.04, 0.70 }, { sx * 1.02, 1.12, zD1 + 0.35 }, 3, "chassis"))
        add(kit.box_part({ 0.66, 0.04, 0.70 }, { sx * 1.02, 1.12, zD2 - 0.35 }, 3, "chassis"))
      end
      -- a front bumper
      add(kit.box_part({ W - 0.04, 0.34, 0.18 }, { 0, 0.62, 2.2 + 0.02 }, 3, "chrome"))
      car.duals = { [zD1] = true, [zD2] = true }
      car.extra_axles = { zD1, zD2 }
      -- THE TRAILER, hitched over the fifth wheel (drawn straight; articulation is the sim's, phase 5)
      local kingpin = fifthZ
      local trL, trW = 16.15, 2.59
      local tFront = kingpin + 0.90                   -- the kingpin sits 0.9 m back from the trailer's nose
      local tBack = tFront - trL
      local tz = (tFront + tBack) * 0.5
      local floorY = 1.25
      local tA1, tA2 = tBack + 1.70, tBack + 2.95     -- the rear tandem
      if trailer == "tanker" then
        -- a tank on a frame: the shell, domed ends, a walkway on top, the frame beneath
        local tr = 1.05
        local shellL = trL - 0.8
        add(kit.cylinder_part(tr, shellL, 16, { 0, floorY + tr + 0.10, tz }, "z", "tank"))
        add(kit.cylinder_part(tr * 0.85, 0.35, 16, { 0, floorY + tr + 0.10, tz + shellL * 0.5 + 0.12 }, "z", "tank"))
        add(kit.cylinder_part(tr * 0.85, 0.35, 16, { 0, floorY + tr + 0.10, tz - shellL * 0.5 - 0.12 }, "z", "tank"))
        add(kit.box_part({ 0.50, 0.05, shellL * 0.8 }, { 0, floorY + 2 * tr + 0.14, tz }, 3, "chassis"))
        for _, sx in ipairs({ 1, -1 }) do add(kit.box_part({ 0.12, 0.22, trL - 1.0 }, { sx * 0.50, floorY - 0.02, tz }, 3, "chassis")) end
      else
        -- a 53 ft box: the body, a floor frame and bottom rails, rear doors, corner posts
        local H = 4.11 - floorY
        add(kit.box_part({ trW, H, trL }, { 0, floorY + H * 0.5, tz }, 3, "trailer"))
        for _, sx in ipairs({ 1, -1 }) do
          add(kit.box_part({ 0.06, 0.16, trL }, { sx * (trW * 0.5 + 0.01), floorY + 0.06, tz }, 3, "chassis"))
          add(kit.box_part({ 0.08, H, 0.08 }, { sx * (trW * 0.5 - 0.02), floorY + H * 0.5, tBack + 0.02 }, 3, "chassis"))
          add(kit.box_part({ trW * 0.5 - 0.10, H - 0.30, 0.02 }, { sx * trW * 0.25, floorY + H * 0.5, tBack - 0.01 }, 3, "door"))
        end
        add(kit.box_part({ trW - 0.2, 0.10, trL - 0.4 }, { 0, floorY - 0.07, tz }, 3, "chassis"))
      end
      -- LANDING GEAR (two legs and a crank) ahead of the trailer's middle, and the rear bogie's rails
      for _, sx in ipairs({ 1, -1 }) do
        add(kit.box_part({ 0.10, floorY - 0.10, 0.10 }, { sx * 0.60, (floorY - 0.10) * 0.5 + 0.08, tFront - 3.5 }, 3, "chassis"))
        add(kit.box_part({ 0.24, 0.05, 0.24 }, { sx * 0.60, 0.06, tFront - 3.5 }, 3, "chassis"))
        add(kit.box_part({ 0.12, 0.22, 3.2 }, { sx * 0.48, floorY - 0.24, (tA1 + tA2) * 0.5 }, 3, "chassis"))
        add(kit.box_part({ 0.62, 0.62, 0.02 }, { sx * 1.02, 0.55, tA1 - 0.62 }, 3, "trim"))   -- mudflaps
      end
      -- the under-ride (ICC) bar at the tail
      add(kit.box_part({ trW - 0.3, 0.12, 0.12 }, { 0, 0.55, tBack + 0.25 }, 3, "chassis"))
      for _, sx in ipairs({ 1, -1 }) do add(kit.box_part({ 0.08, 0.60, 0.08 }, { sx * 0.8, 0.85, tBack + 0.25 }, 3, "chassis")) end
      car.trailer_axles = { tA2, tA1 }
      car.duals[tA1] = true
      car.duals[tA2] = true
    end }
  P.wheel_w = 0.28
  P.track_half = 1.02                 -- wheel centres from the centreline, whatever the body width there
  return P
end
S.semi = function() return semi("box") end
S.semi_tanker = function() return semi("tanker") end

-- DETAIL AFTER SUBDIVISION: the gasket round each window (a flat rubber band at body level), the glass
-- pushed in behind it (a dark reveal), and the grille set into the nose. Added to the SMOOTH mesh, so no
-- later smoothing can pull these centimetre features out of shape -- they stay exactly as built.
function kit.detail(S, P)
  local W = P.windows or {}
  local function setIn(group, spec)
    local sel = S:select{ group = group }
    if #sel == 0 then return end
    spec = spec or {}
    local before = S:face_count()
    S:inset_region(sel, spec.gasket or 0.016)
    local band = {}
    for f = before, S:face_count() - 1 do band[#band + 1] = f end
    S:assign(band, { mat = "gasket", group = group .. "_gasket" })
    local before2 = S:face_count()
    S:extrude(sel, -(spec.depth or 0.012))
    local walls = {}
    for f = before2, S:face_count() - 1 do walls[#walls + 1] = f end
    S:assign(walls, { mat = "trim", group = group .. "_reveal" })
  end
  setIn("win_front", W.windshield)
  setIn("win_back", W.back)
  setIn("win_side_r", W.side)
  setIn("win_side_l", W.side)
  setIn("win_rear", W.back)
  return S
end

-- The body cage (the unsubdivided poly) and its finished form (subdivided + detailed), for inspection
-- and tests.
function kit.poly(P, level)
  local B = kit.body(P)
  return B, kit.detail(B:subdivide(level or 0), P)
end

-- The whole car at `level` subdivisions, as { parts = { [mat] = Mesh }, wheels = {...}, size = {W,H,L} }
-- in body-centre space (origin at the middle of the bounding box, the ground at y = -H/2).
function kit.build(P, level, opts)
  opts = opts or {}
  local B = kit.body(P)
  level = level or P.level or 0
  local S = kit.detail(level > 0 and B:subdivide(level) or B:copy(), P)
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
  -- FACETED (Glenn: "I do like how the city bus's low poly style looks"): at level 0 every facet is flat
  -- shaded; a subdivided body smooths within 40 degrees
  local parts = S:to_parts{ autosmooth = level > 0 and 40 or 8 }
  -- wheels: round, at the axles, just inside the body sides
  local r = P.wheel_r
  local ww = P.wheel_w
  local wheels = {}
  local axles = {}
  for _, zc in ipairs(P.axles) do axles[#axles + 1] = zc end
  for _, zc in ipairs(car.extra_axles or {}) do axles[#axles + 1] = zc end
  for _, zc in ipairs(car.trailer_axles or {}) do axles[#axles + 1] = zc end
  local duals = car.duals or {}
  for _, zc in ipairs(axles) do
    local hw = math.min(P.half_width(math.max(-P.length * 0.5, math.min(P.length * 0.5, zc))), P.width * 0.5)
    local wx = P.track_half or (hw - ww * 0.5 - 0.035)
    for _, sx in ipairs({ 1, -1 }) do
      wheels[#wheels + 1] = { pos = { sx * wx, r - cy, zc - cz }, radius = r, width = ww,
                              front = zc == P.axles[#P.axles], dual = duals[zc] or false }
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
  local N = (level or 0) > 0 and 24 or 14
  -- tyre: outer tread band with rounded shoulders (rings along the axle), capped by the sidewalls
  local T = poly.loft{ sections = { ring(rr, N), ring(r * 0.93, N), ring(r, N), ring(r, N), ring(r * 0.93, N), ring(rr, N) },
                       stations = { -w2, -w2 * 0.92, -w2 * 0.55, w2 * 0.55, w2 * 0.92, w2 }, cap_start = false, cap_end = false }
  local Tm = (level or 0) > 0 and T:subdivide(level) or T
  -- the loft ran along z: turn it onto the x axis
  Tm:map(function(x, y, z) return z, y, -x end)
  local rim = poly.loft{ sections = { ring(rr * 1.0, N), ring(rr * 0.95, N), ring(rr * 0.35, N), ring(rr * 0.2, N) },
                         stations = { w2 * 0.60, w2 * 0.70, w2 * 0.62, w2 * 0.75 }, cap_start = true, cap_end = true,
                         cap_crease = 1 }
  local Rm = (level or 0) > 0 and rim:subdivide(level) or rim
  Rm:map(function(x, y, z) return z, y, -x end)
  local sm = (level or 0) > 0 and 50 or 8
  local tp = Tm:to_parts{ autosmooth = sm }
  local rp = Rm:to_parts{ autosmooth = sm }
  return { tyre = tp.body, rim = rp.body }
end

return kit
