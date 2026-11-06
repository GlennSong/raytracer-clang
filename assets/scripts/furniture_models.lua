-- furniture_models.lua -- THE FURNITURE KIT'S MODELS (#191; Glenn, 2026-11-06: "port the furniture to use the new
-- meshbuilder in lua"). One recipe per kit piece, by its name (furniture.pieces()): `function(variant) -> piece`,
-- built over the mesh words. Read by the engine (scripting/furniture_models_lua.h) and modelled once per
-- (piece, variant), then drawn instanced.
--
-- PIECE SPACE: x across the piece (centred), z out from the wall it backs onto (0 = against the wall), y up from the
-- floor. A piece returns
--   size        the footprint the placer reserves {x, height, z}
--   solid       the placer adds its box to the collider (default true)
--   collider_h  the collider's height when not size[2] (a tap is not a wall)
--   colliders   the parts that block, when not the whole footprint: { {centre, size}, ... }
--   parts       a mesh per finish: wood, fabric, hard (painted / plastic), metal, ceramic, light (lit: emissive)
--
-- VARIANT: bits 0-2 the fabric, 3-4 the wood, the rest (`style`, variant >> 5) the recipe's own dice.

-- VECTORS AND COLOURS: {x, y, z} tables that add and scale (v * k, a + b, a - b)
local Vmt = {}
local function V(x, y, z) return setmetatable({ x, y, z }, Vmt) end
Vmt.__add = function(a, b) return V(a[1] + b[1], a[2] + b[2], a[3] + b[3]) end
Vmt.__sub = function(a, b) return V(a[1] - b[1], a[2] - b[2], a[3] - b[3]) end
Vmt.__mul = function(a, k)
  if type(a) == "number" then a, k = k, a end
  return V(a[1] * k, a[2] * k, a[3] * k)
end
local function grey(k) return V(k, k, k) end
local PI = math.pi

-- THE SWATCH BOOKS. Wood: oak, walnut, birch, ebonised. Fabric: grey, navy, sage, rust, oat, charcoal, mustard, teal.
local WOOD = { V(0.66, 0.50, 0.33), V(0.40, 0.27, 0.18), V(0.82, 0.72, 0.56), V(0.24, 0.18, 0.15) }
local FABRIC = { V(0.48, 0.50, 0.53), V(0.20, 0.26, 0.40), V(0.45, 0.53, 0.42), V(0.62, 0.34, 0.24),
                 V(0.74, 0.68, 0.58), V(0.22, 0.22, 0.24), V(0.72, 0.56, 0.22), V(0.20, 0.42, 0.44) }
local WHITE, BLACK, STEEL, CHROME = V(0.90, 0.90, 0.88), V(0.07, 0.07, 0.08), V(0.62, 0.64, 0.67), V(0.85, 0.86, 0.88)
local PORCELAIN, LINEN = V(0.95, 0.95, 0.93), V(0.93, 0.92, 0.88)
-- the beach's cloths (a lounger's pad, an umbrella, a towel): red, blue, yellow, teal, orange, navy, pink, green
local BEACH = { V(0.80, 0.18, 0.15), V(0.15, 0.35, 0.75), V(0.92, 0.75, 0.18), V(0.10, 0.55, 0.55),
                V(0.90, 0.45, 0.12), V(0.10, 0.15, 0.35), V(0.85, 0.40, 0.55), V(0.25, 0.60, 0.30) }

local function fabricOf(v) return FABRIC[(v & 7) + 1] end
local function woodOf(v) return WOOD[((v >> 3) & 3) + 1] end
local function styleOf(v) return v >> 5 end
-- a bar's timber: walnut or ebonised, by the wood bits
local function barWood(v) return ((v >> 3) & 1) ~= 0 and WOOD[2] or WOOD[4] * 1.2 end
-- one of a swatch list by a draw in [0, 1)
local function pick(list, x) return list[math.floor(x * #list) % #list + 1] end
-- 32-bit seed arithmetic, as the C++ recipes' uint32 seeds wrapped
local function seed(x) return x & 0xffffffff end

-- THE MAKER: a piece under construction, its meshes by finish, and the shape words placed in piece space.
local function maker()
  local k = { size = V(1, 1, 1), solid = true, collider_h = 0, lists = {} }
  local function put(part, m)
    local l = k.lists[part]
    if not l then l = {}; k.lists[part] = l end
    l[#l + 1] = m
  end
  k.put = put
  -- A rounded box centred at c: every edge rounded to r (sharp under 3 mm; the round in `segs` steps only past
  -- 12 mm, where a bevel shows).
  function k.box(part, c, size, r, col, segs)
    put(part, mesh.translate(mesh.paint(mesh.rounded_box(size, r, { segs = r > 0.012 and (segs or 2) or 1, uv = 2 }), col), c))
  end
  -- The same, turned `ang` radians about +x through its own centre (a reclined back cushion).
  function k.tilted(part, c, size, r, ang, col)
    put(part, mesh.translate(mesh.rotate_x(mesh.paint(mesh.rounded_box(size, r, { segs = 2, uv = 2 }), col), ang), c))
  end
  -- A straight round rod from a to b, its ends open (a cap would not show at these radii): a circle swept along a line.
  function k.rod(part, a, b, r, col, sides)
    put(part, mesh.sweep(mesh.profile.circle(r, sides or 10), mesh.path.line(a, b), { caps = false, color = col }))
  end
  -- A turned shape: the profile {r, y}, bottom to top up the outside, revolved about +y and stood at c -- squashed in z
  -- by `sz` (an oval bowl). A profile run top to bottom faces in (a shade's lining).
  function k.turned(part, c, prof, col, sz, segs)
    local m = mesh.revolve(mesh.profile.open(prof), { segs = segs or 20, color = col })
    if sz and sz ~= 1 then m = mesh.scale(m, { 1, 1, sz }) end
    put(part, mesh.translate(m, c))
  end
  -- A square leg of width w standing h tall on its foot.
  function k.leg(part, foot, h, w, col)
    k.box(part, foot + V(0, h * 0.5, 0), V(w, h, w), 0.006, col, 1)
  end
  -- A drawer or door FRONT on a face at z = zFace: a slightly rounded panel proud 1 cm, and a bar handle (upright for
  -- a door).
  function k.front(part, x0, x1, y0, y1, zFace, col, handle, vertical)
    local gap = 0.004
    k.box(part, V((x0 + x1) * 0.5, (y0 + y1) * 0.5, zFace + 0.009), V(x1 - x0 - gap, y1 - y0 - gap, 0.018), 0.004, col, 1)
    local hz = zFace + 0.03
    if vertical then
      local hx, hy = x0 + (x1 - x0) * 0.85, (y0 + y1) * 0.5
      k.rod("metal", V(hx, hy - 0.12, hz), V(hx, hy + 0.12, hz), 0.007, handle, 8)
    else
      local hy, hx = y1 - math.min(0.06, (y1 - y0) * 0.3), (x0 + x1) * 0.5
      k.rod("metal", V(hx - 0.09, hy, hz), V(hx + 0.09, hy, hz), 0.007, handle, 8)
    end
  end
  -- A TAPERED POST from a (its foot) to b: a rounded square `w` wide at b, `wFoot` at a (a chair's or table's leg,
  -- splayed when a and b do not stand over each other).
  function k.post(part, a, b, w, wFoot, col)
    local up = math.abs(b[2] - a[2]) > 0.5 * math.abs(b[1] - a[1]) + 0.5 * math.abs(b[3] - a[3]) and { 0, 0, 1 } or { 0, 1, 0 }
    put(part, mesh.sweep(mesh.profile.rounded_rect(w, w, w * 0.18, 2), mesh.path.line(a, b),
                         { scale = { wFoot / w, 1 }, up = up, color = col }))
  end
  -- A BENT MEMBER along a smooth curve THROUGH `pts` (a chair's back leg running on up into its back post; a bowed
  -- rail): a rounded rectangle w (across) x h (along `up`), scaled along the curve by `scale` (k, {from, to} or fn(t)).
  function k.bent(part, pts, w, h, col, o)
    o = o or {}
    put(part, mesh.sweep(mesh.profile.rounded_rect(w, h, math.min(w, h) * 0.25, 2), mesh.path.spline(pts, { count = o.count or 12 }),
                         { scale = o.scale, up = o.up or { 0, 1, 0 }, color = col }))
  end
  -- A TURNED LEG standing at `foot`, h tall: r at the top tapering to rFoot (a sofa's, a bed's).
  function k.turned_leg(part, foot, h, r, rFoot, col)
    k.turned(part, foot, { { 0.0, 0.0 }, { rFoot, 0.0 }, { r, h }, { 0.0, h } }, col, 1, 12)
  end
  -- THE CAGE: rounded rectangles in plan, ring by ring up y, skinned (poly.loft) and subdivided -- a cushion, a tub,
  -- a bowl. A ring is {w, d, r, y, dx, dz} (dx, dz its centre's offset); rings may run up the outside and back down
  -- the inside (a basin). o: level (subdivision steps, 1), segs (each plan corner's steps, 2), seams = {{y, sharpness}}
  -- (a piped seam, a crisp rim: 1 soft, 3 crisp), cap_start / cap_end (true), shape = fn(x, y, z) -> x, y, z (every
  -- point moved after subdividing), tilt (radians about +x through c), smooth (the autosmooth angle, 50).
  function k.cage(part, c, rings, col, o)
    o = o or {}
    local secs, st = {}, {}
    for i, g in ipairs(rings) do
      local hx, hz, dx, dz = g[1] * 0.5, g[2] * 0.5, g[5] or 0, g[6] or 0
      -- the loft runs up its own z: a ring's plan z is the section's -y (a proper turn, so the faces stay outward)
      secs[i] = poly.section({ { dx - hx, -dz - hz }, { dx + hx, -dz - hz }, { dx + hx, -dz + hz }, { dx - hx, -dz + hz } },
                             g[3], o.segs or 2)
      st[i] = g[4]
    end
    local P = poly.loft{ sections = secs, stations = st, cap_start = o.cap_start ~= false, cap_end = o.cap_end ~= false,
                         cap_crease = 0 }
    P:assign(P:select{}, { mat = part, color = col })
    for _, s in ipairs(o.seams or {}) do P:crease_ring(s[1], s[2]) end
    local S = (o.level or 1) > 0 and P:subdivide(o.level or 1) or P
    local shape = o.shape
    S:map(function(x, y, z)
      if shape then return shape(x, z, -y) end
      return x, z, -y
    end)
    local m = S:to_parts{ autosmooth = o.smooth or 50, uv_scale = 0.5 }[part]
    if o.tilt then m = mesh.rotate_x(m, o.tilt) end
    put(part, mesh.translate(m, c))
  end
  -- A CUSHION centred at c: a top and a bottom panel sewn to a band round the sides, the seams piped. size {x, y, z};
  -- the panels face +-y, or +-z with face = "z" (a back cushion, a headboard). o: r (plan corner), roll (how far in
  -- the panel's edge rolls over), crown (the top panel's rise at its middle; crown_under the bottom's), belly (the
  -- sides bowing out), pinch (0..1, the edges squeezed thin: a pillow), bow (the whole cushion curved, its sides
  -- toward +y / +z: an office chair's back wrapping round), piping (the seams' sharpness, 0 none), warp = fn(u, v, w)
  -- -> dx, dy, dz in metres (u, v, w the point's place, -1..1, across, through the panels, along), tilt, level, segs.
  function k.cushion(part, c, size, col, o)
    o = o or {}
    local up = o.face == "z"
    local sx, sy, sz = size[1], up and size[3] or size[2], up and size[2] or size[3]
    local hx, hy, hz = sx * 0.5, sy * 0.5, sz * 0.5
    local roll = math.min(o.roll or hy * 0.7, hx * 0.45, hz * 0.45)
    local r = math.min(o.r or 0.05, hx * 0.95, hz * 0.95)
    local ri = math.max(r - roll, math.min(r, 0.012))
    local seam = hy - math.min(roll, hy) * 0.45
    local crown, under, belly, pinch, bow = o.crown or 0, o.crown_under or (o.crown or 0) * 0.3, o.belly or 0,
                                            o.pinch or 0, o.bow or 0
    local warp = o.warp
    local piping = o.piping or 0
    k.cage(part, c, { { sx - 2 * roll, sz - 2 * roll, ri, -hy }, { sx, sz, r, -seam }, { sx, sz, r, seam },
                      { sx - 2 * roll, sz - 2 * roll, ri, hy } }, col, {
      level = o.level or 1, segs = o.segs or 2, tilt = o.tilt,
      seams = piping > 0 and { { -seam, piping }, { seam, piping } } or nil,
      shape = function(x, y, z)
        local u, v, w = x / hx, y / hy, z / hz
        local flat = math.max(0, 1 - u * u) * math.max(0, 1 - w * w)
        y = y + (v > 0 and crown or under) * v * flat
        local side = math.max(0, 1 - v * v)
        x, z = x + belly * u * side, z + belly * w * side
        if pinch > 0 then y = y * (1 - pinch * math.max(u * u, w * w) ^ 2) end
        y = y + bow * u * u
        if warp then
          local dx, dy, dz = warp(u, v, w)
          x, y, z = x + dx, y + dy, z + dz
        end
        if up then return x, -z, y end
        return x, y, z
      end })
  end
  -- A CLOTH LAID OVER A SUPPORT (a duvet, a turned-down sheet, a bedspread): a slab o.t thick, its middle at height
  -- o.top, lying flat over |x| <= o.hw from o.z0 to o.zfoot (or o.z1 with no foot fold), folding down over those
  -- edges round radius o.rho and hanging o.hang below the fold, the hanging hem waved o.wave.
  function k.drape(part, col, o)
    local t, rho, hw, top = o.t, o.rho, o.hw, o.top
    local arc = rho * PI * 0.5
    local dmax = arc + o.hang
    local function ticks(a, b, coarse, folds)
      local raw = {}
      for i = 0, math.ceil((b - a) / coarse) do raw[#raw + 1] = math.min(a + i * coarse, b) end
      for _, f in ipairs(folds) do
        for i = 0, 7 do raw[#raw + 1] = f[1] + (f[2] - f[1]) * i / 7 end
      end
      table.sort(raw)
      local out = {}
      for _, x in ipairs(raw) do
        if x >= a - 1e-9 and x <= b + 1e-9 and (#out == 0 or x - out[#out] > 0.01) then out[#out + 1] = x end
      end
      if b - out[#out] > 1e-6 then out[#out] = b end
      return out
    end
    local X = hw + dmax
    local xs = ticks(-X, X, 0.12, { { -hw - arc - 0.03, -hw + 0.02 }, { hw - 0.02, hw + arc + 0.03 } })
    local z1 = o.zfoot and (o.zfoot + dmax) or o.z1
    local zs = ticks(o.z0, z1, 0.15, o.zfoot and { { o.zfoot - 0.02, o.zfoot + arc + 0.03 } } or {})
    local ring = {}
    for i = #xs, 1, -1 do ring[#ring + 1] = { xs[i], t * 0.5 } end
    for i = 1, #xs do ring[#ring + 1] = { xs[i], -t * 0.5 } end
    local secs = {}
    for i = 1, #zs do secs[i] = ring end
    local P = poly.loft{ sections = secs, stations = zs, cap_start = true, cap_end = true, cap_crease = 3 }
    P:assign(P:select{}, { mat = part, color = col })
    local wave = o.wave or 0
    P:map(function(x, yl, z)
      local ex = math.max(0, math.abs(x) - hw)
      local ez = o.zfoot and math.max(0, z - o.zfoot) or 0
      local d = math.sqrt(ex * ex + ez * ez)
      if d < 1e-9 then return x, top + yl, z end
      local dx, dz = (x < 0 and -ex or ex) / d, ez / d
      local bx, bz = ex > 0 and (x < 0 and -hw or hw) or x, ez > 0 and o.zfoot or z
      local h, y
      if d <= arc then
        local a = d / rho
        h, y = (rho + yl) * math.sin(a), top - rho + (rho + yl) * math.cos(a)
      else
        h, y = rho + yl, top - rho - (d - arc)
      end
      local s = math.min(1, math.max(0, (d - arc) / math.max(o.hang, 1e-6)))
      h = h + wave * s * (math.abs(dx) * math.sin(z * 11.0) + dz * math.sin(x * 11.0 + 1.3))
      return bx + dx * h, y, bz + dz * h
    end)
    put(part, P:to_parts{ autosmooth = 60, uv_scale = 0.5 }[part])
  end
  -- THE PIECE: each finish's parts merged into one mesh
  function k.done()
    local parts = {}
    for part, l in pairs(k.lists) do parts[part] = mesh.merge(l) end
    return { size = k.size, solid = k.solid, collider_h = k.collider_h, colliders = k.colliders, parts = parts }
  end
  return k
end

local M = {}
furniture_models = M

-- the swivel chair's five-star base and the stacking pedestal (DeskPod, Cubicle, MeetingTable)
local SWIVEL_BASE = { { 0.26, 0.0 }, { 0.26, 0.03 }, { 0.03, 0.06 }, { 0.025, 0.44 }, { 0.0, 0.45 } }

M.desk = function(v)
  -- 1.6 x 0.8 top on a pair of steel sled frames, a three-drawer pedestal under the right end.
  local k, wood = maker(), woodOf(v)
  k.size = V(1.6, 0.75, 0.8)
  k.box("wood", V(0, 0.73, 0.4), V(1.6, 0.03, 0.8), 0.006, wood, 1)
  for _, x in ipairs({ -0.74, 0.74 }) do
    k.rod("metal", V(x, 0.02, 0.06), V(x, 0.02, 0.74), 0.016, BLACK)   -- foot
    k.rod("metal", V(x, 0.70, 0.06), V(x, 0.70, 0.74), 0.016, BLACK)   -- top rail
    k.rod("metal", V(x, 0.02, 0.10), V(x, 0.70, 0.10), 0.016, BLACK)   -- uprights
    k.rod("metal", V(x, 0.02, 0.70), V(x, 0.70, 0.70), 0.016, BLACK)
  end
  k.rod("metal", V(-0.74, 0.62, 0.08), V(0.74, 0.62, 0.08), 0.012, BLACK)   -- back stretcher
  local px0, px1 = 0.22, 0.68
  k.box("hard", V((px0 + px1) * 0.5, 0.36, 0.38), V(px1 - px0, 0.56, 0.62), 0.006, WHITE, 1)
  for d = 0, 2 do
    local y0 = 0.10 + d * 0.18
    k.front("hard", px0, px1, y0, y0 + 0.18, 0.69, WHITE, CHROME)
  end
  return k.done()
end

M.office_chair = function(v)
  -- Five-star base on casters, a gas lift, a contoured seat with a waterfall front, a back that wraps round with a
  -- lumbar swell, armrest pads.
  local k, fabric = maker(), fabricOf(v)
  k.size = V(0.66, 1.05, 0.66)
  k.solid = false
  local c = V(0, 0, 0.33)
  for i = 0, 4 do
    local a = 2 * PI * i / 5 + 0.3
    local tip = c + V(0.30 * math.cos(a), 0.07, 0.30 * math.sin(a))
    k.bent("hard", { c + V(0, 0.10, 0), c + V(0.16 * math.cos(a), 0.095, 0.16 * math.sin(a)), tip }, 0.03, 0.026, BLACK,
           { count = 6, scale = { 1.2, 0.8 } })                                       -- the star's arm
    k.turned("hard", tip - V(0, 0.07, 0), { { 0.0, 0.0 }, { 0.028, 0.008 }, { 0.032, 0.03 }, { 0.028, 0.055 }, { 0.0, 0.065 } },
             BLACK, 1, 10)   -- caster
  end
  k.turned("metal", c, { { 0.04, 0.08 }, { 0.04, 0.14 }, { 0.024, 0.15 }, { 0.024, 0.42 }, { 0.03, 0.44 } }, CHROME, 1, 16)
  k.box("hard", c + V(0, 0.445, 0.0), V(0.30, 0.03, 0.30), 0.01, BLACK, 1)          -- the seat's mechanism
  k.cushion("fabric", c + V(0, 0.49, 0.02), V(0.50, 0.08, 0.48), fabric * 0.75,
            { r = 0.08, roll = 0.03, crown = 0.012,
              warp = function(u, v, w)                                                -- dished, the front rolled down
                return 0, -0.012 * math.max(0, 1 - u * u) * math.max(0, 1 - w * w) * (v > 0 and 1 or 0)
                          - 0.03 * math.max(0, w - 0.55) / 0.45 * (v + 1) * 0.5, 0
              end })
  k.cushion("fabric", c + V(0, 0.80, -0.21), V(0.46, 0.54, 0.07), fabric * 0.75,
            { face = "z", r = 0.09, roll = 0.025, crown = 0.01, bow = 0.045, tilt = -0.18,
              warp = function(u, v, w)                                                -- the lumbar swell low down
                return 0, 0.02 * math.exp(-((w - 0.45) / 0.3) ^ 2), 0
              end })
  k.bent("hard", { c + V(0, 0.445, -0.10), c + V(0, 0.47, -0.22), c + V(0, 0.62, -0.25) }, 0.05, 0.016, BLACK,
         { count = 8 })                                                              -- the back's spine
  for _, x in ipairs({ -0.27, 0.27 }) do
    k.bent("hard", { c + V(x * 0.7, 0.45, 0.0), c + V(x, 0.50, 0.0), c + V(x, 0.64, 0.0) }, 0.03, 0.02, BLACK, { count = 8 })
    k.cushion("hard", c + V(x, 0.655, 0.02), V(0.075, 0.03, 0.25), BLACK, { r = 0.03, roll = 0.012, crown = 0.004 })
  end
  return k.done()
end

M.monitor = function(v)
  -- The screen faces +z (the chair); the stand stands BEHIND it, toward the wall (Glenn: "the computer monitors are
  -- backwards" -- the neck was on the screen side).
  local k = maker()
  k.size = V(0.62, 0.48, 0.22)
  k.solid = false
  k.box("hard", V(0, 0.008, 0.08), V(0.24, 0.016, 0.16), 0.008, BLACK, 1)
  k.box("hard", V(0, 0.17, 0.06), V(0.05, 0.30, 0.025), 0.008, BLACK, 1)
  k.box("hard", V(0, 0.30, 0.085), V(0.60, 0.36, 0.025), 0.006, BLACK, 1)
  k.box("metal", V(0, 0.30, 0.0985), V(0.58, 0.34, 0.004), 0.0, V(0.02, 0.025, 0.03), 1)   -- the glass
  k.box("hard", V(0, 0.012, 0.19), V(0.44, 0.02, 0.13), 0.006, BLACK, 1)                    -- keyboard
  return k.done()
end

M.filing_cabinet = function(v)
  local k = maker()
  k.size = V(0.46, 1.05, 0.6)
  k.box("hard", V(0, 0.525, 0.30), V(0.46, 1.05, 0.58), 0.006, STEEL * 0.85, 1)
  for d = 0, 2 do
    local y0 = 0.04 + d * 0.33
    k.front("hard", -0.22, 0.22, y0, y0 + 0.33, 0.59, STEEL * 0.9, CHROME)
  end
  return k.done()
end

local function makeBed(k, w, top, z0, zf, duvetFrom, fabric, pillows, pz)
  local mh = 0.22
  k.cushion("fabric", V(0, top - mh * 0.5, (z0 + zf) * 0.5), V(w, mh, zf - z0), LINEN,
            { r = 0.06, roll = 0.03, crown = 0.008, piping = 2.5 })                       -- the sheeted mattress
  local t, rho = 0.045, 0.05
  local hw = w * 0.5 - rho + t * 0.5 + 0.006
  k.drape("fabric", fabric, { t = t, rho = rho, hw = hw, top = top + t * 0.5 + 0.004, z0 = duvetFrom,
                              zfoot = zf - rho + t * 0.5 + 0.006, hang = 0.17, wave = 0.008 })   -- the duvet
  local ts, rs = 0.022, rho + t + 0.01
  k.drape("fabric", LINEN * 1.03, { t = ts, rho = rs, hw = hw - t * 0.5 - 0.006 + ts * 0.5,
                                    top = top + t + 0.004 + ts * 0.5 + 0.003, z0 = duvetFrom - 0.004, z1 = duvetFrom + 0.26,
                                    hang = 0.08 })                                      -- the turned-down sheet
  for _, x in ipairs(pillows) do
    k.cushion("fabric", V(x, top + 0.075, pz), V(w > 1.2 and 0.66 or 0.62, 0.15, 0.40), LINEN * 1.02,
              { r = 0.07, roll = 0.04, crown = 0.02, pinch = 0.55, belly = 0.01, tilt = 0.22 })
  end
end

M.bed = function(v)
  -- A made double: a timber frame on legs, an upholstered headboard (plain, or channel-tufted by the style), the
  -- mattress, the duvet draped over it, the turned-back sheet, two pillows (Glenn: "the beds don't have sheets on
  -- them").
  local k, wood, fabric = maker(), woodOf(v), fabricOf(v)
  k.size = V(1.66, 1.05, 2.12)
  k.box("wood", V(0, 0.22, 1.06), V(1.66, 0.14, 2.10), 0.012, wood, 1)
  for _, x in ipairs({ -0.76, 0.76 }) do
    for _, z in ipairs({ 0.08, 2.02 }) do k.turned_leg("wood", V(x, 0, z), 0.16, 0.03, 0.02, wood * 0.9) end
  end
  if styleOf(v) % 2 == 1 then
    for i = 0, 5 do
      k.cushion("fabric", V(-0.83 + 0.1383 + i * 0.2767, 0.60, 0.05), V(0.2767, 0.96, 0.10), fabric * 0.8,
                { face = "z", r = 0.03, roll = 0.03, crown = 0.02, segs = 1 })              -- channels
    end
  else
    k.cushion("fabric", V(0, 0.60, 0.05), V(1.66, 0.96, 0.10), fabric * 0.8,
              { face = "z", r = 0.05, roll = 0.04, crown = 0.012, piping = 1.5 })
  end
  makeBed(k, 1.56, 0.51, 0.08, 2.06, 0.64, fabric, { -0.38, 0.38 }, 0.33)
  return k.done()
end

M.nightstand = function(v)
  local k, wood = maker(), woodOf(v)
  k.size = V(0.46, 0.95, 0.40)
  k.box("wood", V(0, 0.30, 0.20), V(0.46, 0.44, 0.40), 0.008, wood, 1)
  for _, x in ipairs({ -0.19, 0.19 }) do
    for _, z in ipairs({ 0.04, 0.36 }) do k.leg("wood", V(x, 0, z), 0.08, 0.035, wood * 0.9) end
  end
  k.front("wood", -0.21, 0.21, 0.30, 0.50, 0.40, wood * 0.95, CHROME)
  -- the lamp: a turned base and a drum shade
  k.turned("ceramic", V(0.06, 0.52, 0.18), { { 0.0, 0.0 }, { 0.07, 0.0 }, { 0.075, 0.02 }, { 0.05, 0.12 }, { 0.06, 0.20 },
                                             { 0.015, 0.24 }, { 0.0, 0.25 } }, PORCELAIN * 0.92, 1, 18)
  k.turned("fabric", V(0.06, 0.74, 0.18), { { 0.13, 0.0 }, { 0.13, 0.002 }, { 0.10, 0.17 }, { 0.098, 0.172 } }, LINEN, 1, 20)
  return k.done()
end

M.wardrobe = function(v)
  local k, wood = maker(), woodOf(v)
  k.size = V(1.2, 2.05, 0.6)
  k.box("wood", V(0, 1.04, 0.30), V(1.2, 1.98, 0.58), 0.008, wood, 1)
  k.box("wood", V(0, 0.03, 0.29), V(1.16, 0.06, 0.54), 0.004, wood * 0.7, 1)   -- plinth
  k.front("wood", -0.59, 0.0, 0.08, 2.02, 0.59, wood * 1.04, CHROME, true)
  k.front("wood", 0.0, 0.59, 0.08, 2.02, 0.59, wood * 1.04, CHROME, true)
  return k.done()
end

M.sofa = function(v)
  -- Three seat cushions and three back cushions, crowned and piped, on an upholstered base with rolled arms and a
  -- back, on turned legs (#191: soft shapes from the cushion cage).
  local k, wood, fabric = maker(), woodOf(v), fabricOf(v)
  k.size = V(2.16, 0.86, 0.94)
  k.cushion("fabric", V(0, 0.24, 0.48), V(2.0, 0.22, 0.86), fabric * 0.9, { r = 0.04, roll = 0.03, belly = 0.006 })
  for _, x in ipairs({ -1.0, 1.0 }) do
    k.cushion("fabric", V(x, 0.37, 0.47), V(0.18, 0.48, 0.92), fabric * 0.95,
              { r = 0.06, roll = 0.07, crown = 0.015, piping = 1.5, segs = 2 })   -- the rolled arm
  end
  for i = 0, 2 do
    local x = -0.62 + i * 0.62
    k.cushion("fabric", V(x, 0.42, 0.55), V(0.60, 0.15, 0.66), fabric,
              { r = 0.05, crown = 0.03, belly = 0.01, piping = 2 })
    k.cushion("fabric", V(x, 0.68, 0.19), V(0.60, 0.42, 0.18), fabric * 1.03,
              { face = "z", r = 0.06, crown = 0.035, belly = 0.01, piping = 2, tilt = -0.16 })
  end
  k.cushion("fabric", V(0, 0.56, 0.08), V(2.0, 0.42, 0.14), fabric * 0.9, { face = "z", r = 0.04, roll = 0.04, crown = 0.01 })
  for _, x in ipairs({ -0.98, 0.98 }) do
    for _, z in ipairs({ 0.08, 0.86 }) do k.turned_leg("wood", V(x, 0, z), 0.13, 0.024, 0.015, wood * 0.8) end
  end
  return k.done()
end

M.coffee_table = function(v)
  local k, wood = maker(), woodOf(v)
  k.size = V(1.1, 0.42, 0.6)
  k.box("wood", V(0, 0.40, 0.30), V(1.1, 0.035, 0.6), 0.012, wood, 1)
  k.box("wood", V(0, 0.14, 0.30), V(1.0, 0.02, 0.5), 0.004, wood * 0.9, 1)   -- the lower shelf
  for _, x in ipairs({ -0.50, 0.50 }) do
    for _, z in ipairs({ 0.05, 0.55 }) do k.leg("wood", V(x, 0, z), 0.385, 0.04, wood * 0.85) end
  end
  return k.done()
end

M.tv_unit = function(v)
  local k, wood = maker(), woodOf(v)
  k.size = V(1.6, 1.25, 0.42)
  k.box("wood", V(0, 0.30, 0.21), V(1.6, 0.42, 0.42), 0.008, wood, 1)
  for _, x in ipairs({ -0.74, 0.74 }) do k.leg("metal", V(x, 0, 0.21), 0.10, 0.03, BLACK) end
  k.front("wood", -0.78, -0.26, 0.11, 0.49, 0.42, wood * 1.04, BLACK)
  k.front("wood", 0.26, 0.78, 0.11, 0.49, 0.42, wood * 1.04, BLACK)
  k.box("hard", V(0, 0.53, 0.20), V(0.30, 0.02, 0.20), 0.004, BLACK, 1)     -- TV foot
  k.box("hard", V(0, 0.58, 0.20), V(0.06, 0.10, 0.03), 0.004, BLACK, 1)
  k.box("hard", V(0, 0.93, 0.20), V(1.25, 0.72, 0.04), 0.004, BLACK, 1)     -- the set
  k.box("metal", V(0, 0.93, 0.2205), V(1.22, 0.69, 0.002), 0.0, V(0.02, 0.02, 0.025), 1)
  return k.done()
end

-- One 0.6 m module of a counter run: the carcass on its kick plinth, a door (or drawers), a slice of worktop -- and in
-- it the sink (a stainless bowl and a tap) or the hob (black glass, four rings).
local function kitchenModule(v, kind)
  local k, wood, fabric, style = maker(), woodOf(v), fabricOf(v), styleOf(v)
  k.size = V(0.6, kind == "sink" and 1.18 or 0.92, 0.62)   -- the sink's tap stands tall
  k.collider_h = 0.92                                     -- but the run collides as a counter
  local door = (style & 1) ~= 0 and WHITE or fabric * 0.75 + grey(0.12)
  k.box("hard", V(0, 0.05, 0.27), V(0.6, 0.10, 0.52), 0.0, BLACK * 2.0, 1)
  k.box("hard", V(0, 0.48, 0.29), V(0.6, 0.76, 0.58), 0.0, WHITE * 0.95, 1)
  if kind == "hob" then
    for d = 0, 2 do k.front("hard", -0.3, 0.3, 0.10 + d * 0.253, 0.10 + (d + 1) * 0.253, 0.58, door, CHROME) end
  else
    k.front("hard", -0.3, 0.3, 0.10, 0.86, 0.58, door, CHROME, true)
  end
  local stone = (style & 2) ~= 0
  k.box(stone and "ceramic" or "wood", V(0, 0.88, 0.31), V(0.6, 0.04, 0.62), 0.003, stone and V(0.20, 0.20, 0.21) or wood * 1.05, 1)
  if kind == "sink" then
    k.box("metal", V(0, 0.896, 0.33), V(0.46, 0.004, 0.40), 0.0, CHROME, 1)
    k.box("metal", V(0, 0.81, 0.33), V(0.40, 0.16, 0.34), 0.02, STEEL * 0.6, 2)   -- the bowl's floor
    k.turned("metal", V(0, 0.90, 0.10), { { 0.025, 0.0 }, { 0.022, 0.03 }, { 0.012, 0.04 }, { 0.012, 0.26 }, { 0.0, 0.27 } },
             CHROME, 1, 12)
    k.rod("metal", V(0, 1.15, 0.10), V(0, 1.12, 0.26), 0.012, CHROME, 10)
  elseif kind == "hob" then
    k.box("metal", V(0, 0.902, 0.32), V(0.56, 0.006, 0.50), 0.004, V(0.03, 0.03, 0.035), 1)
    for _, x in ipairs({ -0.13, 0.13 }) do
      for _, z in ipairs({ 0.20, 0.44 }) do
        k.turned("hard", V(x, 0.905, z), { { 0.0, 0.0 }, { 0.085, 0.0 }, { 0.085, 0.002 }, { 0.0, 0.002 } }, V(0.18, 0.18, 0.19), 1, 20)
      end
    end
  end
  return k.done()
end
M.kitchen_base = function(v) return kitchenModule(v, "base") end
M.kitchen_sink = function(v) return kitchenModule(v, "sink") end
M.kitchen_hob = function(v) return kitchenModule(v, "hob") end

M.kitchen_tall = function(v)
  -- The fridge tower: full height, a long handle.
  local k = maker()
  k.size = V(0.6, 2.0, 0.64)
  k.box("hard", V(0, 1.0, 0.31), V(0.6, 2.0, 0.62), 0.004, WHITE * 0.95, 1)
  k.front("hard", -0.3, 0.3, 0.80, 1.98, 0.62, STEEL, CHROME, true)
  k.front("hard", -0.3, 0.3, 0.10, 0.80, 0.62, STEEL, CHROME, true)
  return k.done()
end

M.kitchen_wall = function(v)
  -- A wall cupboard, hung at 1.45 m.
  local k, fabric, style = maker(), fabricOf(v), styleOf(v)
  k.size = V(0.6, 2.15, 0.35)
  k.solid = false
  k.box("hard", V(0, 1.80, 0.17), V(0.6, 0.70, 0.34), 0.0, WHITE * 0.95, 1)
  local door = (style & 1) ~= 0 and WHITE or fabric * 0.75 + grey(0.12)
  k.front("hard", -0.3, 0.3, 1.45, 2.15, 0.34, door, CHROME, true)
  return k.done()
end

M.dining_table = function(v)
  -- A 1.4 x 0.85 top with a softened edge over an apron, on legs tapering to the floor.
  local k, wood = maker(), woodOf(v)
  k.size = V(1.4, 0.76, 0.85)
  k.put("wood", mesh.translate(mesh.extrude(mesh.profile.rounded_rect(1.4, 0.85, 0.03, 3), 0.04,
                                            { bevel = 0.01, bevel_segs = 2, color = wood }), V(0, 0.72, 0.425)))
  k.box("wood", V(0, 0.68, 0.425), V(1.26, 0.08, 0.72), 0.004, wood * 0.9, 1)   -- the apron
  for _, x in ipairs({ -0.64, 0.64 }) do
    for _, z in ipairs({ 0.07, 0.78 }) do k.post("wood", V(x, 0, z), V(x, 0.72, z), 0.058, 0.036, wood * 0.95) end
  end
  return k.done()
end

M.dining_chair = function(v)
  -- Tapered front legs; the back legs run on up into the back posts, raked back; a drop-in seat pad on rails; the
  -- back by the style: two bowed rails, three spindles, or an upholstered pad.
  local k, wood, fabric = maker(), woodOf(v), fabricOf(v)
  k.size = V(0.46, 0.88, 0.50)
  k.solid = false
  local style = styleOf(v) % 3
  for _, x in ipairs({ -0.19, 0.19 }) do
    k.post("wood", V(x, 0, 0.455), V(x, 0.44, 0.445), 0.036, 0.026, wood)
    k.bent("wood", { V(x, 0, 0.035), V(x, 0.44, 0.065), V(x, 0.88, 0.02) }, 0.034, 0.036, wood,
           { up = { 0, 0, 1 }, scale = function(t) return 0.8 + 0.2 * math.sin(t * PI) end })
    k.rod("wood", V(x, 0.16, 0.07), V(x, 0.16, 0.43), 0.009, wood * 0.95, 8)      -- side stretchers
  end
  for _, s in ipairs({ { 0, 0.415, 0.45, 0.38, 0.02 }, { 0, 0.415, 0.065, 0.38, 0.02 } }) do
    k.box("wood", V(s[1], s[2], s[3]), V(s[4], 0.06, s[5]), 0.004, wood * 0.97, 1)   -- front and back rails
  end
  for _, x in ipairs({ -0.19, 0.19 }) do k.box("wood", V(x, 0.415, 0.26), V(0.02, 0.06, 0.37), 0.004, wood * 0.97, 1) end
  k.cushion("fabric", V(0, 0.465, 0.265), V(0.42, 0.045, 0.40), fabric, { r = 0.04, roll = 0.02, crown = 0.012, piping = 1 })
  local function postZ(y) return 0.065 - 0.045 * (y - 0.44) / 0.44 end
  local function rail(y, h)
    local z = postZ(y)
    k.bent("wood", { V(-0.19, y, z), V(0, y, z - 0.035), V(0.19, y, z) }, 0.02, h, wood, { count = 10 })
  end
  if style == 0 then
    rail(0.62, 0.05); rail(0.81, 0.07)
  elseif style == 1 then
    rail(0.83, 0.07)
    for _, x in ipairs({ -0.09, 0, 0.09 }) do
      local zb = postZ(0.47) - 0.035 * (1 - (x / 0.19) ^ 2) + 0.005
      local zt = postZ(0.80) - 0.035 * (1 - (x / 0.19) ^ 2)
      k.rod("wood", V(x, 0.47, zb), V(x, 0.80, zt), 0.009, wood, 8)                -- spindles
    end
    rail(0.48, 0.03)
  else
    k.cushion("fabric", V(0, 0.70, postZ(0.70) + 0.01), V(0.38, 0.30, 0.04), fabric,
              { face = "z", r = 0.04, roll = 0.015, crown = 0.008, bow = -0.02, piping = 1, tilt = -0.1 })
  end
  return k.done()
end

M.bathtub = function(v)
  -- A built-in tub moulded in one piece: the apron up to a rolled rim, the basin sloping down from it to a flat floor,
  -- the back end raked for lying in, the chrome mixer at the tap end.
  local k = maker()
  k.size = V(1.70, 0.70, 0.75)   -- to the mixer's top
  local W, D, H, t = 1.70, 0.75, 0.56, 0.07
  local z = D * 0.5
  k.cage("ceramic", V(0, 0, z), {
    { W, D, 0.02, 0.0 }, { W, D, 0.02, H - 0.025 }, { W - 0.004, D - 0.004, 0.03, H },
    { W - 2 * t, D - 2 * t, 0.13, H }, { W - 2 * t - 0.02, D - 2 * t - 0.02, 0.13, H - 0.04, -0.01 },
    { W - 2 * t - 0.22, D - 2 * t - 0.08, 0.16, 0.25, -0.07 }, { W - 2 * t - 0.42, D - 2 * t - 0.16, 0.14, 0.07, -0.12 },
  }, PORCELAIN, { level = 1, segs = 3, seams = { { 0.0, 1e6 }, { H - 0.025, 2 }, { H, 1 } } })
  k.turned("metal", V(-W * 0.5 + 0.12, H, z), { { 0.02, 0.0 }, { 0.018, 0.12 }, { 0.0, 0.13 } }, CHROME, 1, 12)
  k.bent("metal", { V(-W * 0.5 + 0.12, H + 0.11, z), V(-W * 0.5 + 0.2, H + 0.13, z), V(-W * 0.5 + 0.27, H + 0.08, z) },
         0.02, 0.02, CHROME, { count = 8 })                                           -- the spout
  k.turned("metal", V(-W * 0.5 + 0.30, 0.074, z), { { 0.0, 0.0 }, { 0.03, 0.0 }, { 0.03, 0.004 }, { 0.0, 0.005 } },
           CHROME, 1, 12)                                                             -- the plug
  return k.done()
end

M.toilet = function(v)
  -- A close-coupled WC: the pedestal narrowing to its foot and flaring to an egg-shaped bowl, hollow, its rim rolled;
  -- the seat; a lid down by the style (or none: a public WC's open seat); the cistern with its lid and flush button.
  local k = maker()
  k.size = V(0.40, 0.80, 0.70)
  k.solid = false
  k.box("ceramic", V(0, 0.59, 0.10), V(0.37, 0.34, 0.16), 0.03, PORCELAIN, 2)        -- cistern
  k.box("ceramic", V(0, 0.775, 0.10), V(0.385, 0.035, 0.175), 0.012, PORCELAIN, 2)   -- and its lid
  k.turned("metal", V(0.0, 0.792, 0.10), { { 0.0, 0.0 }, { 0.025, 0.0 }, { 0.024, 0.008 }, { 0.0, 0.009 } }, CHROME, 1, 14)
  local zb = 0.43
  k.box("ceramic", V(0, 0.30, 0.20), V(0.20, 0.30, 0.10), 0.03, PORCELAIN, 2)        -- the neck to the cistern
  k.cage("ceramic", V(0, 0, zb), {
    { 0.22, 0.30, 0.10, 0.0, 0, -0.03 }, { 0.19, 0.27, 0.09, 0.15, 0, -0.04 }, { 0.33, 0.44, 0.16, 0.31, 0, 0.0 },
    { 0.36, 0.48, 0.17, 0.385, 0, 0.0 }, { 0.36, 0.48, 0.17, 0.40, 0, 0.0 }, { 0.27, 0.37, 0.13, 0.40, 0, 0.005 },
    { 0.23, 0.31, 0.11, 0.33, 0, 0.01 }, { 0.10, 0.12, 0.045, 0.20, 0, -0.02 },
  }, PORCELAIN, { level = 1, segs = 3, seams = { { 0.0, 1e6 }, { 0.40, 1 } } })
  k.cage("hard", V(0, 0, zb), {
    { 0.25, 0.33, 0.12, 0.402, 0, 0.005 }, { 0.37, 0.47, 0.175, 0.402, 0, 0.0 }, { 0.37, 0.47, 0.175, 0.418, 0, 0.0 },
    { 0.25, 0.33, 0.12, 0.418, 0, 0.005 }, { 0.25, 0.33, 0.12, 0.402, 0, 0.005 },
  }, WHITE, { level = 1, segs = 3, cap_start = false, cap_end = false })              -- the seat
  if styleOf(v) % 2 == 0 then
    k.cage("hard", V(0, 0, zb), { { 0.37, 0.47, 0.175, 0.419, 0, 0.0 }, { 0.37, 0.47, 0.175, 0.428, 0, 0.0 },
                                  { 0.35, 0.45, 0.165, 0.437, 0, 0.0 } }, WHITE,
           { level = 1, segs = 3, seams = { { 0.419, 1 } } })                         -- the lid, down
  end
  return k.done()
end

M.vanity = function(v)
  -- The cabinet with a shaker door (a frame round a recessed panel), a ceramic top with a vessel basin and a tall
  -- mixer, and above it a round-cornered mirror on a framed backing.
  local k, wood = maker(), woodOf(v)
  k.size = V(0.80, 1.95, 0.50)
  k.box("wood", V(0, 0.47, 0.24), V(0.80, 0.56, 0.48), 0.006, wood, 1)
  local x0, x1, y0, y1, f = -0.395, 0.395, 0.20, 0.74, 0.055
  k.box("wood", V(0, (y0 + y1) * 0.5, 0.484), V(x1 - x0 - 2 * f, y1 - y0 - 2 * f, 0.008), 0.002, wood * 1.02, 1)  -- panel
  for _, s in ipairs({ { (x0 + x1) * 0.5, y0 + f * 0.5, x1 - x0, f }, { (x0 + x1) * 0.5, y1 - f * 0.5, x1 - x0, f },
                       { x0 + f * 0.5, (y0 + y1) * 0.5, f, y1 - y0 - 2 * f }, { x1 - f * 0.5, (y0 + y1) * 0.5, f, y1 - y0 - 2 * f } }) do
    k.box("wood", V(s[1], s[2], 0.491), V(s[3], s[4], 0.022), 0.003, wood * 1.04, 1)                     -- the frame
  end
  k.rod("metal", V(-0.09, y1 - 0.03, 0.52), V(0.09, y1 - 0.03, 0.52), 0.007, CHROME, 8)
  k.box("ceramic", V(0, 0.765, 0.25), V(0.80, 0.03, 0.50), 0.006, PORCELAIN, 1)
  k.turned("ceramic", V(0, 0.78, 0.27), { { 0.0, 0.0 }, { 0.18, 0.0 }, { 0.20, 0.09 }, { 0.19, 0.10 }, { 0.15, 0.03 }, { 0.0, 0.03 } },
           PORCELAIN, 0.8, 24)                                                  -- vessel basin
  k.turned("metal", V(0, 0.78, 0.05), { { 0.022, 0.0 }, { 0.018, 0.20 }, { 0.0, 0.21 } }, CHROME, 1, 12)
  k.bent("metal", { V(0, 0.975, 0.05), V(0, 0.99, 0.11), V(0, 0.955, 0.17) }, 0.02, 0.02, CHROME, { count = 8 })
  -- the mirror: a panel in the wall's plane, its corners rounded, laid flat and stood up (centred through its depth
  -- so the turn's sense does not matter)
  local function pane(w, h, r, d, col)
    local m = mesh.extrude(mesh.profile.rounded_rect(w, h, r, 4), d, { color = col })
    return mesh.rotate_x(mesh.translate(m, V(0, -d * 0.5, 0)), PI * 0.5)
  end
  k.put("hard", mesh.translate(pane(0.74, 0.92, 0.07, 0.016, BLACK), V(0, 1.45, 0.010)))
  k.put("metal", mesh.translate(pane(0.69, 0.87, 0.05, 0.006, CHROME), V(0, 1.45, 0.021)))
  return k.done()
end

M.lounge_chair = function(v)
  local k, wood, fabric = maker(), woodOf(v), fabricOf(v)
  k.size = V(0.84, 0.80, 0.84)
  k.cushion("fabric", V(0, 0.30, 0.44), V(0.84, 0.24, 0.80), fabric, { r = 0.05, roll = 0.04, belly = 0.006 })
  k.cushion("fabric", V(0, 0.46, 0.50), V(0.56, 0.12, 0.62), fabric * 1.04, { r = 0.05, crown = 0.03, belly = 0.01, piping = 2 })
  k.cushion("fabric", V(0, 0.58, 0.12), V(0.84, 0.48, 0.16), fabric * 0.96,
            { face = "z", r = 0.06, roll = 0.05, crown = 0.025, bow = 0.03, piping = 1.5, tilt = -0.12 })
  for _, x in ipairs({ -0.36, 0.36 }) do
    k.cushion("fabric", V(x, 0.50, 0.46), V(0.13, 0.24, 0.74), fabric * 0.96, { r = 0.05, roll = 0.05, crown = 0.012, piping = 1.5 })
  end
  for _, x in ipairs({ -0.36, 0.36 }) do
    for _, z in ipairs({ 0.10, 0.78 }) do k.turned_leg("wood", V(x, 0, z), 0.18, 0.02, 0.012, wood) end
  end
  return k.done()
end

-- A made bed's soft goods over a mattress `w` wide, its top at `top`, from z0 to zf (the foot): the sheeted mattress,
-- the duvet draped over the sides and the foot, the top sheet turned back over the duvet's head, the pillows at `pz`.

M.planter = function(v)
  local k = maker()
  k.size = V(0.9, 1.3, 0.9)
  k.turned("ceramic", V(0, 0, 0.45), { { 0.0, 0.0 }, { 0.22, 0.0 }, { 0.28, 0.42 }, { 0.29, 0.45 }, { 0.0, 0.45 } },
           V(0.30, 0.30, 0.31), 1, 20)
  -- the shrub: a lumpy ball of leaves
  local leaves = mesh.displace(mesh.icosphere(2), { amp = 0.25, freq = 3.0, seed = 7 })
  k.put("fabric", mesh.translate(mesh.paint(mesh.scale(leaves, { 0.36, 0.45, 0.36 }), V(0.20, 0.36, 0.16)), V(0, 0.85, 0.45)))
  return k.done()
end

-- abstract canvases: a ground and up to three fields
local PICTURE_PALETTES = {
  { V(0.93, 0.90, 0.84), V(0.85, 0.36, 0.22), V(0.16, 0.24, 0.40), V(0.92, 0.72, 0.30) },
  { V(0.20, 0.22, 0.26), V(0.84, 0.80, 0.72), V(0.70, 0.30, 0.30), V(0.40, 0.56, 0.62) },
  { V(0.88, 0.86, 0.80), V(0.36, 0.52, 0.40), V(0.70, 0.62, 0.44), V(0.22, 0.30, 0.26) },
  { V(0.96, 0.94, 0.90), V(0.10, 0.10, 0.12), V(0.86, 0.20, 0.18), V(0.20, 0.36, 0.70) },
  { V(0.82, 0.74, 0.64), V(0.58, 0.38, 0.30), V(0.92, 0.84, 0.70), V(0.40, 0.30, 0.26) },
  { V(0.30, 0.40, 0.52), V(0.70, 0.80, 0.86), V(0.94, 0.86, 0.60), V(0.16, 0.20, 0.30) },
}
M.picture = function(v)
  -- WALL ART (Glenn: "the walls are bare"): a framed canvas hung at eye height -- an abstract of two to four colour
  -- fields on a ground, in one of three sizes and three frames, the palette by the variant.
  local k, wood, style = maker(), woodOf(v), styleOf(v)
  local sizes = { { 0.60, 0.80 }, { 1.00, 0.70 }, { 1.40, 0.90 } }
  local W, H = sizes[style % 3 + 1][1], sizes[style % 3 + 1][2]
  local cy = 1.55
  k.size = V(W + 0.1, cy + H * 0.5 + 0.06, 0.05)
  k.solid = false
  local frames = { V(0.07, 0.07, 0.08), V(0.92, 0.91, 0.88), V(0.0, 0.0, 0.0) }
  local fi = (style // 3) % 3
  local frame = fi == 2 and wood or frames[fi + 1]
  local fm = fi == 2 and "wood" or "hard"
  local fw = 0.04
  k.box(fm, V(0, cy + H * 0.5 + fw * 0.5, 0.02), V(W + 2 * fw, fw, 0.04), 0.004, frame, 1)
  k.box(fm, V(0, cy - H * 0.5 - fw * 0.5, 0.02), V(W + 2 * fw, fw, 0.04), 0.004, frame, 1)
  k.box(fm, V(-W * 0.5 - fw * 0.5, cy, 0.02), V(fw, H, 0.04), 0.004, frame, 1)
  k.box(fm, V(W * 0.5 + fw * 0.5, cy, 0.02), V(fw, H, 0.04), 0.004, frame, 1)
  local pal = PICTURE_PALETTES[seed(v ~ (style * 7)) % 6 + 1]
  k.box("hard", V(0, cy, 0.022), V(W, H, 0.012), 0.0, pal[1], 1)   -- the ground
  -- the fields: seeded rectangles, kept inside the canvas
  local r = rng.new(seed(v * 2654435761 + style * 40503))
  local fields = 2 + math.floor(r:next() * 3)
  for f = 0, fields - 1 do
    local fwid = W * (0.18 + 0.45 * r:next())
    local fht = H * (0.15 + 0.5 * r:next())
    local fx = (r:next() - 0.5) * (W - fwid)
    local fyy = (r:next() - 0.5) * (H - fht)
    k.box("hard", V(fx, cy + fyy, 0.029 + 0.001 * f), V(fwid, fht, 0.002), 0.0, pal[2 + f % 3], 1)
  end
  return k.done()
end

M.shelving = function(v)
  -- A CLOSET's shelves: two white uprights, four shelves of folded clothes and boxes, a hanging rail of garments
  -- down one half.
  local k = maker()
  k.size = V(1.2, 2.0, 0.5)
  for _, x in ipairs({ -0.58, 0.58 }) do k.box("hard", V(x, 1.0, 0.25), V(0.03, 2.0, 0.48), 0.003, WHITE, 1) end
  for _, y in ipairs({ 0.30, 0.80, 1.40, 1.90 }) do k.box("hard", V(0.29, y, 0.25), V(0.56, 0.025, 0.46), 0.002, WHITE, 1) end
  k.box("hard", V(0, 1.98, 0.25), V(1.16, 0.025, 0.48), 0.002, WHITE, 1)
  local r = rng.new(seed(v * 747796405 + 1))
  for _, y in ipairs({ 0.31, 0.81, 1.41 }) do
    local x = 0.05
    while x < 0.5 do
      local w = 0.14 + 0.12 * r:next()
      local hh = 0.10 + 0.18 * r:next()
      if x + w > 0.54 then break end
      k.box("fabric", V(x + w * 0.5, y + 0.013 + hh * 0.5, 0.24), V(w, hh, 0.34), 0.02, pick(FABRIC, r:next()), 1)
      x = x + w + 0.02
    end
  end
  k.rod("metal", V(-0.56, 1.75, 0.25), V(0.0, 1.75, 0.25), 0.012, CHROME, 10)
  for g = 0, 6 do
    local x = -0.52 + g * 0.075
    k.box("fabric", V(x, 1.30, 0.25), V(0.018, 0.84, 0.42), 0.008, FABRIC[(v + g * 3) % 8 + 1])
  end
  return k.done()
end

M.rug = function(v)
  local k, fabric = maker(), fabricOf(v)
  k.size = V(2.0, 0.04, 1.4)
  k.solid = false
  k.box("fabric", V(0, 0.026, 0.7), V(2.0, 0.012, 1.4), 0.005, fabric * 0.85, 1)   -- lifted clear of the floor finish
  k.box("fabric", V(0, 0.028, 0.7), V(1.76, 0.012, 1.16), 0.004, LINEN * 0.9, 1)
  k.box("fabric", V(0, 0.030, 0.7), V(1.5, 0.012, 0.9), 0.004, fabric, 1)
  return k.done()
end

M.desk_pod = function(v)
  -- BENCHING, running from the windows into the room: three desks (style bit 4: two) either side of a low fabric
  -- screen on the x = 0 line, a monitor, keyboard and chair at each. Footprint 3.1 (x) by 4.8 (z) for six, 3.2 for four.
  local k, wood, fabric, style = maker(), woodOf(v), fabricOf(v), styleOf(v)
  local four = (style & 4) ~= 0
  local L = four and 3.2 or 4.8
  k.size = V(3.1, 1.38, L)
  -- the desks and the screen down their middle collide; the chairs either side are seats (library spots)
  k.colliders = { { V(0, 0.375, L * 0.5), V(1.6, 0.75, L) }, { V(0, 1.05, L * 0.5), V(0.04, 0.62, L) } }
  k.box("wood", V(0, 0.735, L * 0.5), V(1.6, 0.03, L), 0.004, wood, 1)
  for _, x in ipairs({ -0.76, 0.76 }) do
    for _, z in ipairs({ 0.05, L * 0.5, L - 0.05 }) do k.leg("metal", V(x, 0, z), 0.72, 0.04, WHITE * 0.9) end
  end
  k.box("fabric", V(0, 1.05, L * 0.5), V(0.04, 0.62, L), 0.012, fabric * 0.7 + grey(0.12))
  for _, z in ipairs(four and { 0.8, 2.4 } or { 0.8, 2.4, 4.0 }) do
    for _, s in ipairs({ -1.0, 1.0 }) do
      local xm = s * 0.22
      k.box("hard", V(xm, 0.76, z), V(0.16, 0.012, 0.22), 0.004, BLACK, 1)              -- foot
      k.box("hard", V(xm, 0.92, z), V(0.02, 0.30, 0.04), 0.004, BLACK, 1)               -- neck
      k.box("hard", V(xm + s * 0.02, 1.06, z), V(0.022, 0.34, 0.58), 0.004, BLACK, 1)   -- screen
      k.box("hard", V(s * 0.55, 0.75, z), V(0.14, 0.02, 0.44), 0.004, BLACK, 1)         -- keyboard
      local xch = s * 1.15
      k.turned("metal", V(xch, 0, z), SWIVEL_BASE, BLACK, 1, 10)
      k.box("fabric", V(xch, 0.48, z), V(0.46, 0.07, 0.48), 0.03, fabric * 0.75)
      k.box("fabric", V(xch + s * 0.22, 0.80, z), V(0.05, 0.50, 0.44), 0.025, fabric * 0.75)
    end
  end
  return k.done()
end

M.cubicle = function(v)
  -- A CUBICLE: fabric partitions on three sides (1.4 m), an L-shaped worktop, a pedestal, the chair and the monitor;
  -- open at +z.
  local k, wood, fabric = maker(), woodOf(v), fabricOf(v)
  k.size = V(2.4, 1.45, 2.4)
  -- (the panels and the worktops collide, not the floor inside: you walk in and sit at it)
  k.colliders = { { V(0, 0.70, 0.03), V(2.4, 1.4, 0.06) }, { V(-1.17, 0.70, 1.2), V(0.06, 1.4, 2.4) },
                  { V(1.17, 0.70, 1.2), V(0.06, 1.4, 2.4) }, { V(0, 0.37, 0.40), V(2.2, 0.74, 0.70) },
                  { V(-0.80, 0.37, 1.15), V(0.60, 0.74, 0.85) } }
  local panel = fabric * 0.6 + grey(0.18)
  k.box("fabric", V(0, 0.70, 0.03), V(2.4, 1.4, 0.06), 0.015, panel)
  for _, x in ipairs({ -1.17, 1.17 }) do k.box("fabric", V(x, 0.70, 1.2), V(0.06, 1.4, 2.4), 0.015, panel) end
  for _, x in ipairs({ -1.17, 1.17 }) do k.box("metal", V(x, 1.405, 1.2), V(0.07, 0.012, 2.4), 0.0, STEEL, 1) end
  k.box("metal", V(0, 1.405, 0.03), V(2.4, 0.012, 0.07), 0.0, STEEL, 1)
  k.box("wood", V(0, 0.73, 0.40), V(2.2, 0.03, 0.70), 0.004, wood, 1)                -- the back run
  k.box("wood", V(-0.80, 0.73, 1.15), V(0.60, 0.03, 0.85), 0.004, wood, 1)           -- the return
  k.box("hard", V(0.75, 0.33, 0.40), V(0.42, 0.62, 0.58), 0.004, STEEL * 0.85, 1)    -- the pedestal
  k.box("hard", V(0, 0.76, 0.22), V(0.22, 0.012, 0.16), 0.004, BLACK, 1)
  k.box("hard", V(0, 0.92, 0.20), V(0.04, 0.30, 0.02), 0.004, BLACK, 1)
  k.box("hard", V(0, 1.06, 0.23), V(0.58, 0.34, 0.022), 0.004, BLACK, 1)
  k.turned("metal", V(0, 0, 1.2), SWIVEL_BASE, BLACK, 1, 10)
  k.box("fabric", V(0, 0.48, 1.2), V(0.48, 0.07, 0.46), 0.03, fabric * 0.75)
  k.box("fabric", V(0, 0.80, 1.43), V(0.44, 0.50, 0.05), 0.025, fabric * 0.75)
  return k.done()
end

M.meeting_table = function(v)
  -- A meeting table for six: a long top on two pedestal legs, three chairs a side.
  local k, wood, fabric = maker(), woodOf(v), fabricOf(v)
  k.size = V(3.0, 0.95, 2.4)
  k.colliders = { { V(0, 0.375, 1.2), V(2.6, 0.75, 1.1) } }   -- the table; its chairs are seats
  k.box("wood", V(0, 0.735, 1.2), V(2.6, 0.04, 1.1), 0.015, wood, 1)
  for _, x in ipairs({ -0.8, 0.8 }) do k.box("metal", V(x, 0.36, 1.2), V(0.12, 0.72, 0.7), 0.01, STEEL * 0.7, 1) end
  for _, x in ipairs({ -0.85, 0.0, 0.85 }) do
    for _, s in ipairs({ -1.0, 1.0 }) do
      local zch = 1.2 + s * 0.85
      k.turned("metal", V(x, 0, zch), { { 0.24, 0.0 }, { 0.24, 0.03 }, { 0.03, 0.06 }, { 0.025, 0.44 }, { 0.0, 0.45 } }, CHROME, 1, 10)
      k.box("fabric", V(x, 0.48, zch), V(0.46, 0.07, 0.44), 0.03, fabric)
      k.box("fabric", V(x, 0.78, zch + s * 0.21), V(0.44, 0.46, 0.05), 0.025, fabric)
    end
  end
  return k.done()
end

M.whiteboard = function(v)
  -- style bit 2: a LECTURE HALL's board, 4 m x 1.3 m (Glenn: "the lecture hall should have a bigger whiteboard")
  local k, style = maker(), styleOf(v)
  local wide = (style & 4) ~= 0
  local bw, bh, cy = wide and 4.0 or 1.9, wide and 1.3 or 1.04, wide and 1.55 or 1.45
  k.size = V(bw, wide and 2.3 or 2.1, 0.06)
  k.solid = false
  k.box("metal", V(0, cy, 0.02), V(bw - 0.06, bh, 0.03), 0.004, CHROME, 1)
  k.box("ceramic", V(0, cy, 0.037), V(bw - 0.12, bh - 0.06, 0.004), 0.0, PORCELAIN, 1)
  k.box("metal", V(0, cy - bh * 0.5 - 0.01, 0.05), V(bw * 0.74, 0.02, 0.06), 0.004, CHROME, 1)   -- the pen tray
  k.box("hard", V(-0.3, 1.6, 0.04), V(0.6, 0.012, 0.002), 0.0, V(0.15, 0.25, 0.6), 1)            -- a scrawl
  k.box("hard", V(0.2, 1.35, 0.04), V(0.8, 0.012, 0.002), 0.0, V(0.6, 0.15, 0.15), 1)
  return k.done()
end

M.shop_counter = function(v)
  -- The SHOP COUNTER: a panelled front, a worktop, the till and a card reader. Faces +z (the customer).
  local k, wood, style = maker(), woodOf(v), styleOf(v)
  k.size = V(1.8, 1.25, 0.7)
  k.collider_h = 1.0
  k.box("wood", V(0, 0.48, 0.35), V(1.8, 0.96, 0.66), 0.006, wood, 1)
  k.box("hard", V(0, 1.0, 0.36), V(1.84, 0.04, 0.72), 0.006, (style & 1) ~= 0 and WHITE or BLACK * 3.0, 1)
  k.box("hard", V(0.4, 1.10, 0.25), V(0.36, 0.16, 0.30), 0.01, BLACK, 1)   -- the till
  k.box("hard", V(0.4, 1.22, 0.22), V(0.32, 0.18, 0.03), 0.004, BLACK, 1)
  k.box("hard", V(-0.3, 1.05, 0.5), V(0.08, 0.06, 0.14), 0.008, BLACK, 1)  -- card reader
  return k.done()
end

M.gondola = function(v)
  -- A GONDOLA: a double-sided run of shelves (the grocery aisle), four shelves a side of boxed and bottled goods.
  -- Runs along x; both faces (+z, -z) stocked.
  local k = maker()
  k.size = V(2.4, 1.6, 1.0)
  k.box("hard", V(0, 0.06, 0.5), V(2.4, 0.12, 0.96), 0.004, BLACK * 3.0, 1)   -- base
  k.box("hard", V(0, 0.85, 0.5), V(2.4, 1.5, 0.04), 0.004, WHITE * 0.9, 1)    -- spine
  local r = rng.new(seed(v * 2246822519 + 3))
  local goods = { V(0.85, 0.20, 0.15), V(0.95, 0.75, 0.15), V(0.20, 0.45, 0.80), V(0.25, 0.65, 0.30),
                  V(0.90, 0.90, 0.88), V(0.55, 0.30, 0.65), V(0.95, 0.50, 0.15), V(0.30, 0.30, 0.32) }
  for _, s in ipairs({ -1.0, 1.0 }) do
    for _, y in ipairs({ 0.14, 0.50, 0.86, 1.22 }) do
      k.box("hard", V(0, y, 0.5 + s * 0.24), V(2.36, 0.02, 0.44), 0.002, WHITE * 0.9, 1)
      local x = -1.15
      while x < 1.1 do
        local w = 0.10 + 0.22 * r:next()
        local hh = 0.14 + 0.16 * r:next()
        if x + w > 1.15 then break end
        k.box("hard", V(x + w * 0.5, y + 0.01 + hh * 0.5, 0.5 + s * 0.26), V(w - 0.01, hh, 0.32), 0.0, pick(goods, r:next()), 1)
        x = x + w
      end
    end
  end
  return k.done()
end

M.wall_shelf = function(v)
  -- WALL SHELVING, one-sided against a wall: five shelves of stock.
  local k, wood = maker(), woodOf(v)
  k.size = V(2.0, 2.1, 0.45)
  k.box("wood", V(0, 1.05, 0.03), V(2.0, 2.1, 0.04), 0.003, wood * 0.9, 1)
  for _, x in ipairs({ -0.98, 0.98 }) do k.box("wood", V(x, 1.05, 0.22), V(0.04, 2.1, 0.44), 0.003, wood, 1) end
  local r = rng.new(seed(v * 3266489917 + 5))
  for _, y in ipairs({ 0.10, 0.50, 0.90, 1.30, 1.70 }) do
    k.box("wood", V(0, y, 0.22), V(1.92, 0.025, 0.42), 0.002, wood, 1)
    local x = -0.92
    while x < 0.9 do
      local w = 0.08 + 0.18 * r:next()
      local hh = 0.12 + 0.2 * r:next()
      if x + w > 0.94 then break end
      k.box("hard", V(x + w * 0.5, y + 0.013 + hh * 0.5, 0.22), V(w - 0.01, hh, 0.30), 0.0, pick(FABRIC, r:next()) * 1.2, 1)
      x = x + w
    end
  end
  return k.done()
end

M.clothes_rack = function(v)
  -- A boutique's CLOTHES RAIL: two uprights, a chrome rail, a dozen garments on hangers.
  local k = maker()
  k.size = V(1.6, 1.65, 0.6)
  k.solid = false
  for _, x in ipairs({ -0.75, 0.75 }) do
    k.rod("metal", V(x, 0.0, 0.3), V(x, 1.55, 0.3), 0.014, CHROME, 10)
    k.box("metal", V(x, 0.01, 0.3), V(0.06, 0.02, 0.55), 0.004, CHROME, 1)
  end
  k.rod("metal", V(-0.75, 1.55, 0.3), V(0.75, 1.55, 0.3), 0.012, CHROME, 10)
  for g = 0, 11 do
    local x = -0.66 + g * 0.12
    local len = 0.6 + 0.35 * (((v + g * 7) % 5) / 4.0)
    k.box("fabric", V(x, 1.50 - len * 0.5, 0.3), V(0.03, len, 0.46), 0.012, FABRIC[(v + g * 3) % 8 + 1] * 1.1)
  end
  return k.done()
end

M.cafe_table = function(v)
  -- A CAFE TABLE for two: a round top on a pedestal, two bentwood-ish chairs.
  local k, wood = maker(), woodOf(v)
  k.size = V(1.5, 0.9, 1.5)
  k.turned("metal", V(0, 0, 0.75), { { 0.24, 0.0 }, { 0.24, 0.02 }, { 0.03, 0.04 }, { 0.03, 0.72 }, { 0.0, 0.73 } }, BLACK, 1, 14)
  k.turned("wood", V(0, 0.72, 0.75), { { 0.0, 0.0 }, { 0.36, 0.0 }, { 0.36, 0.03 }, { 0.0, 0.03 } }, wood, 1, 24)
  for side = 0, 1 do
    local z, s = side == 0 and 0.18 or 1.32, side == 0 and -1.0 or 1.0
    for _, x in ipairs({ -0.18, 0.18 }) do
      for _, dz in ipairs({ -0.16, 0.16 }) do k.leg("wood", V(x, 0, z + dz), 0.44, 0.03, wood * 0.85) end
    end
    k.box("wood", V(0, 0.46, z), V(0.42, 0.04, 0.40), 0.01, wood * 0.9, 1)
    k.box("wood", V(0, 0.72, z + s * 0.19), V(0.40, 0.42, 0.03), 0.01, wood * 0.9, 1)
  end
  return k.done()
end

M.display_case = function(v)
  -- A bakery / jeweller's DISPLAY CASE: a lit glass case on a cabinet, goods on two glass shelves.
  local k, wood = maker(), woodOf(v)
  k.size = V(1.6, 1.25, 0.7)
  k.box("wood", V(0, 0.42, 0.35), V(1.6, 0.84, 0.66), 0.006, wood, 1)
  k.box("hard", V(0, 1.24, 0.35), V(1.6, 0.03, 0.66), 0.004, WHITE, 1)
  for _, x in ipairs({ -0.79, 0.79 }) do k.box("metal", V(x, 1.04, 0.35), V(0.02, 0.38, 0.64), 0.0, CHROME, 1) end
  local r = rng.new(seed(v * 668265263 + 11))
  local treats = { V(0.80, 0.55, 0.30), V(0.95, 0.85, 0.65), V(0.45, 0.25, 0.15), V(0.90, 0.40, 0.45) }
  for _, y in ipairs({ 0.86, 1.04 }) do
    for i = 0, 6 do
      local z = 0.25 + 0.2 * r:next()
      k.turned("hard", V(-0.6 + i * 0.2, y, z), { { 0.0, 0.0 }, { 0.06, 0.0 }, { 0.06, 0.03 }, { 0.0, 0.05 } }, pick(treats, r:next()), 1, 10)
    end
  end
  return k.done()
end

M.drinks_fridge = function(v)
  -- A DRINKS FRIDGE: a tall cabinet, a glass door, five shelves of bottles and cans.
  local k = maker()
  k.size = V(0.8, 2.0, 0.75)
  k.box("hard", V(0, 1.0, 0.37), V(0.8, 2.0, 0.72), 0.006, WHITE * 0.95, 1)
  k.box("hard", V(0, 1.0, 0.72), V(0.70, 1.80, 0.02), 0.0, V(0.05, 0.06, 0.08), 1)   -- the dark interior
  local r = rng.new(seed(v * 2654435761 + 13))
  local cans = { V(0.85, 0.10, 0.10), V(0.10, 0.30, 0.80), V(0.95, 0.85, 0.20), V(0.20, 0.70, 0.30), V(0.90, 0.90, 0.90),
                 V(0.55, 0.20, 0.10) }
  for sh = 0, 4 do
    local y = 0.25 + sh * 0.34
    for i = 0, 5 do
      k.turned("hard", V(-0.27 + i * 0.108, y, 0.62), { { 0.0, 0.0 }, { 0.035, 0.0 }, { 0.035, 0.18 }, { 0.015, 0.24 }, { 0.0, 0.25 } },
               pick(cans, r:next()), 1, 8)
    end
  end
  k.box("metal", V(0, 1.0, 0.745), V(0.74, 1.84, 0.01), 0.0, V(0.55, 0.62, 0.66), 1)   -- the glass
  k.rod("metal", V(0.32, 0.7, 0.77), V(0.32, 1.3, 0.77), 0.01, CHROME, 8)
  return k.done()
end

M.bookcase = function(v)
  -- A BOOKCASE: five shelves of books, spines in every colour, the odd one leaning.
  local k, wood = maker(), woodOf(v)
  k.size = V(1.2, 2.0, 0.35)
  k.box("wood", V(0, 1.0, 0.02), V(1.2, 2.0, 0.03), 0.003, wood * 0.85, 1)
  for _, x in ipairs({ -0.585, 0.585 }) do k.box("wood", V(x, 1.0, 0.18), V(0.03, 2.0, 0.34), 0.003, wood, 1) end
  local r = rng.new(seed(v * 1597334677 + 7))
  -- BOOKS (Glenn: "It would be nice to have some more variety in the books"): cloth and leather in sixteen colours,
  -- every height and depth, gilt or cream title bands on the spine, now and then a stack lying flat. Each placement is
  -- its own bookcase (the library's `variety`: eight designs by position).
  local spines = { V(0.55, 0.12, 0.10), V(0.12, 0.20, 0.45), V(0.15, 0.40, 0.22), V(0.85, 0.80, 0.70),
                   V(0.10, 0.10, 0.10), V(0.75, 0.55, 0.20), V(0.45, 0.30, 0.50), V(0.30, 0.50, 0.55),
                   V(0.40, 0.22, 0.12), V(0.60, 0.38, 0.22), V(0.35, 0.08, 0.10), V(0.08, 0.12, 0.25),
                   V(0.70, 0.62, 0.40), V(0.55, 0.55, 0.52), V(0.20, 0.30, 0.15), V(0.80, 0.30, 0.20) }
  local bands = { V(0.80, 0.65, 0.25), V(0.88, 0.85, 0.75), V(0.08, 0.08, 0.08) }
  for _, y in ipairs({ 0.04, 0.42, 0.80, 1.18, 1.56 }) do
    k.box("wood", V(0, y, 0.18), V(1.14, 0.025, 0.32), 0.002, wood, 1)
    local x = -0.55
    local series = math.floor(r:next() * 16) + 1   -- a shelf often holds a set in one binding
    while x < 0.53 do
      if r:next() < 0.07 and x < 0.30 then   -- a stack lying flat
        local n = 3 + math.floor(r:next() * 3)
        local yy = y + 0.013
        for _ = 1, n do
          local th = 0.025 + 0.02 * r:next()
          local sx = 0.22 + 0.04 * r:next()
          local sz = 0.17 + 0.04 * r:next()
          k.box("hard", V(x + 0.12, yy + th * 0.5, 0.17), V(sx, th, sz), 0.002, pick(spines, r:next()), 1)
          yy = yy + th
        end
        x = x + 0.28
      else
        local w = 0.02 + 0.045 * r:next()
        local hh = 0.18 + 0.13 * r:next()
        local dd = 0.16 + 0.08 * r:next()
        if x + w > 0.56 then break end
        local col
        if r:next() < 0.45 then col = spines[series] * (0.85 + 0.3 * r:next()) else col = pick(spines, r:next()) end
        k.box("hard", V(x + w * 0.5, y + 0.013 + hh * 0.5, 0.29 - dd * 0.5), V(w - 0.003, hh, dd), 0.002, col, 1)
        -- title bands on the spine's face
        if w > 0.026 and r:next() < 0.75 then
          local band = pick(bands, r:next())
          local nb = 1 + math.floor(r:next() * 2)
          for j = 0, nb - 1 do
            local by = y + 0.013 + hh * (0.62 + 0.22 * j + 0.06 * r:next())
            k.box("hard", V(x + w * 0.5, math.min(by, y + 0.013 + hh - 0.02), 0.292), V(w - 0.008, 0.012, 0.004), 0.0, band, 1)
          end
        end
        x = x + w + (r:next() < 0.04 and 0.04 or 0.0)   -- the odd gap where a book is out
      end
    end
  end
  return k.done()
end

M.pallet_rack = function(v)
  -- A big box's PALLET RACKING (Glenn, 2026-10-01: "big box stores like Costco"): a double-sided bay of steel uprights
  -- and orange beams, shelf-height goods on the floor level, shrink-wrapped pallets of cartons on the two levels above.
  -- Runs along x, stocked on both faces.
  local k = maker()
  k.size = V(2.8, 4.4, 2.2)
  k.collider_h = 2.6   -- high enough to wall the aisle; the pallets above are out of reach
  local upright, beam = V(0.16, 0.30, 0.55), V(0.92, 0.42, 0.10)
  for _, x in ipairs({ -1.37, 1.37 }) do
    for _, z in ipairs({ 0.06, 1.1, 2.14 }) do k.box("metal", V(x, 2.2, z), V(0.07, 4.4, 0.07), 0.0, upright, 1) end
  end
  local r = rng.new(seed(v * 2654435761 + 11))
  local carton = { V(0.62, 0.48, 0.32), V(0.70, 0.56, 0.38), V(0.55, 0.42, 0.28), V(0.85, 0.85, 0.82), V(0.30, 0.42, 0.62) }
  for _, y in ipairs({ 1.55, 3.0 }) do
    for _, z in ipairs({ 0.06, 2.14 }) do k.box("metal", V(0, y, z), V(2.74, 0.12, 0.06), 0.0, beam, 1) end
    for side = 0, 1 do
      local zc = side == 0 and 0.58 or 1.62
      for _, x in ipairs({ -0.68, 0.68 }) do
        k.box("wood", V(x, y + 0.13, zc), V(1.2, 0.14, 0.95), 0.0, V(0.72, 0.60, 0.42), 1)   -- pallet
        local hh = 0.7 + 0.55 * r:next()
        k.box("hard", V(x, y + 0.2 + hh * 0.5, zc), V(1.14, hh, 0.9), 0.01, pick(carton, r:next()), 1)
      end
    end
  end
  -- the floor level: shelf-height goods both sides
  for side = 0, 1 do
    local zc = side == 0 and 0.55 or 1.65
    k.box("metal", V(0, 0.08, zc), V(2.7, 0.04, 0.9), 0.0, STEEL * 0.8, 1)
    local x = -1.3
    while x < 1.25 do
      local w = 0.25 + 0.35 * r:next()
      local hh = 0.35 + 0.6 * r:next()
      if x + w > 1.32 then break end
      local col = pick(carton, r:next())
      k.box("hard", V(x + w * 0.5, 0.1 + hh * 0.5, zc), V(w - 0.02, hh, 0.8), 0.0, col * (0.8 + 0.4 * r:next()), 1)
      x = x + w
    end
  end
  return k.done()
end

M.checkout = function(v)
  -- A CHECKOUT LANE: a long counter running out from the front (z), its belt, the register and card reader at the
  -- far end, and the lane's numbered light on a pole.
  local k = maker()
  k.size = V(0.9, 2.3, 3.2)
  k.collider_h = 0.95
  k.box("hard", V(0, 0.45, 1.6), V(0.85, 0.9, 3.2), 0.01, V(0.30, 0.31, 0.34), 2)       -- the counter
  k.box("hard", V(0, 0.915, 1.1), V(0.6, 0.03, 2.0), 0.005, BLACK, 1)                   -- the belt
  k.box("hard", V(0, 0.915, 2.6), V(0.75, 0.03, 1.0), 0.005, STEEL, 1)                  -- bagging well
  k.box("hard", V(0.25, 1.15, 2.3), V(0.30, 0.28, 0.08), 0.01, BLACK, 1)                -- register screen
  k.rod("metal", V(0.25, 0.92, 2.3), V(0.25, 1.02, 2.3), 0.02, STEEL, 8)
  k.box("hard", V(-0.3, 1.0, 2.1), V(0.12, 0.16, 0.08), 0.01, V(0.15, 0.15, 0.17), 1)   -- card reader
  k.rod("metal", V(0.38, 0.9, 0.1), V(0.38, 2.15, 0.1), 0.025, STEEL, 8)                -- the light pole
  k.box("hard", V(0.38, 2.2, 0.1), V(0.3, 0.22, 0.12), 0.01, V(0.95, 0.85, 0.25), 1)    -- its lamp
  return k.done()
end

M.bench = function(v)
  -- A PARK BENCH: timber slats on two cast-iron ends with armrests, a slatted back, seats three. Its back at z = 0, the
  -- seat facing +z.
  local k, wood = maker(), woodOf(v)
  k.size = V(1.80, 0.86, 0.66)
  k.collider_h = 0.46   -- the seat: you walk round the back, not through it
  local iron = V(0.14, 0.15, 0.16)
  for _, x in ipairs({ -0.84, 0.84 }) do
    k.box("metal", V(x, 0.22, 0.10), V(0.06, 0.44, 0.06), 0.01, iron, 1)        -- back leg
    k.box("metal", V(x, 0.22, 0.56), V(0.06, 0.44, 0.06), 0.01, iron, 1)        -- front leg
    k.box("metal", V(x, 0.42, 0.33), V(0.06, 0.05, 0.56), 0.01, iron, 1)        -- seat rail
    k.tilted("metal", V(x, 0.66, 0.08), V(0.06, 0.48, 0.05), 0.01, -0.22, iron) -- back rail
    k.box("metal", V(x, 0.64, 0.42), V(0.07, 0.04, 0.40), 0.01, iron, 1)        -- armrest
    k.box("metal", V(x, 0.53, 0.60), V(0.05, 0.22, 0.04), 0.01, iron, 1)
  end
  for i = 0, 4 do k.box("wood", V(0, 0.455, 0.14 + i * 0.095), V(1.74, 0.03, 0.075), 0.006, wood, 1) end   -- seat slats
  for i = 0, 2 do   -- back slats, leaning back
    k.tilted("wood", V(0, 0.56 + i * 0.11, 0.095 - i * 0.025), V(1.74, 0.075, 0.03), 0.006, -0.22, wood)
  end
  return k.done()
end

M.plaza_bench = function(v)
  -- A PLAZA BENCH: backless, a thick timber top on two stone plinths; sat on from either side (+z is "front").
  local k, wood = maker(), woodOf(v)
  k.size = V(1.80, 0.46, 0.55)
  local stone = V(0.66, 0.64, 0.60)
  for _, x in ipairs({ -0.70, 0.70 }) do k.box("hard", V(x, 0.20, 0.275), V(0.30, 0.40, 0.50), 0.015, stone, 1) end
  k.box("wood", V(0, 0.43, 0.275), V(1.80, 0.06, 0.55), 0.01, wood, 1)
  return k.done()
end

M.bistro_table = function(v)
  -- A BISTRO TABLE: a round top on a cast pedestal (café terraces, café floors; its chairs are their own).
  local k, wood = maker(), woodOf(v)
  k.size = V(0.70, 0.74, 0.70)
  k.turned("metal", V(0, 0, 0.35), { { 0.22, 0.0 }, { 0.22, 0.02 }, { 0.03, 0.05 }, { 0.03, 0.71 }, { 0.0, 0.72 } }, BLACK, 1, 14)
  k.turned("wood", V(0, 0.71, 0.35), { { 0.0, 0.0 }, { 0.35, 0.0 }, { 0.35, 0.03 }, { 0.0, 0.03 } }, wood, 1, 24)
  return k.done()
end

M.bistro_chair = function(v)
  -- A BISTRO CHAIR: a bentwood-style seat and hooped back on four legs. Back at z = 0, facing +z.
  local k, wood = maker(), woodOf(v)
  k.size = V(0.44, 0.84, 0.48)
  k.solid = false
  for _, x in ipairs({ -0.17, 0.17 }) do
    for _, z in ipairs({ 0.06, 0.42 }) do k.leg("wood", V(x, 0, z), 0.45, 0.028, wood * 0.85) end
  end
  k.turned("wood", V(0, 0.45, 0.25), { { 0.0, 0.0 }, { 0.21, 0.0 }, { 0.21, 0.035 }, { 0.0, 0.035 } }, wood * 0.9, 1, 20)
  for _, x in ipairs({ -0.17, 0.17 }) do k.rod("wood", V(x, 0.47, 0.06), V(x * 0.8, 0.84, 0.03), 0.014, wood * 0.85, 8) end
  k.tilted("wood", V(0, 0.76, 0.035), V(0.34, 0.10, 0.025), 0.01, -0.08, wood * 0.9)
  return k.done()
end

M.ceiling_light = function(v)
  -- THE ROOM'S LIGHT (Glenn, 2026-10-02: "actual ceiling light fixtures for where the ambient light comes from"): hung
  -- by its TOP at y = size.y (placed so that is the ceiling). An office's recessed 600 x 1200 panel -- a white frame
  -- round a lit diffuser -- or, with style bit 5, a home's round flush fitting.
  local k = maker()
  k.solid = false
  if (v & 64) ~= 0 then
    -- style bit 6: a bar's PENDANT -- a cord from the ceiling, a dark metal dome, a warm amber bulb under it
    k.size = V(0.36, 0.90, 0.36)
    k.rod("metal", V(0, 0.30, 0.18), V(0, 0.90, 0.18), 0.005, BLACK, 6)
    k.turned("metal", V(0, 0.12, 0.18), { { 0.17, 0.0 }, { 0.16, 0.06 }, { 0.11, 0.14 }, { 0.04, 0.18 }, { 0.0, 0.19 } },
             (v & 128) ~= 0 and V(0.55, 0.42, 0.22) or BLACK * 1.5, 1, 18)
    k.turned("light", V(0, 0.07, 0.18), { { 0.0, 0.0 }, { 0.045, 0.02 }, { 0.05, 0.06 }, { 0.03, 0.09 }, { 0.0, 0.10 } },
             V(1.0, 0.62, 0.30), 1, 12)
  elseif (v & 32) ~= 0 then
    k.size = V(0.42, 0.08, 0.42)
    k.turned("hard", V(0, 0.06, 0.21), { { 0.0, 0.0 }, { 0.20, 0.0 }, { 0.21, 0.02 }, { 0.0, 0.02 } }, WHITE, 1, 24)
    k.turned("light", V(0, 0.0, 0.21), { { 0.0, 0.0 }, { 0.18, 0.0 }, { 0.19, 0.03 }, { 0.17, 0.06 }, { 0.0, 0.06 } },
             V(1.0, 0.95, 0.85), 1, 24)
  else
    k.size = V(0.62, 0.05, 1.22)
    k.box("hard", V(0, 0.035, 0.61), V(0.62, 0.03, 1.22), 0.0, WHITE, 1)                    -- the frame
    k.box("light", V(0, 0.012, 0.61), V(0.56, 0.02, 1.16), 0.0, V(1.0, 0.98, 0.94), 1)      -- the diffuser
  end
  return k.done()
end

-- ---- GOODS (the furniture library, M3): small things the dressing pass stands on anchors. Each sits on y = 0 (the
-- surface), centred on x = 0, z = size.z / 2. The variant's style bits pick the colourway.
M.desk_lamp = function(v)
  local k = maker()
  k.size = V(0.22, 0.46, 0.22)
  k.solid = false
  local shade = (v & 32) ~= 0 and BLACK * 2.0 or V(0.75, 0.22, 0.18)
  k.turned("metal", V(0, 0, 0.11), { { 0.0, 0.0 }, { 0.09, 0.0 }, { 0.09, 0.02 }, { 0.02, 0.03 }, { 0.0, 0.03 } }, BLACK * 2.0, 1, 16)
  k.rod("metal", V(0, 0.02, 0.11), V(0.0, 0.30, 0.13), 0.008, CHROME, 8)
  k.rod("metal", V(0, 0.30, 0.13), V(0.0, 0.40, 0.04), 0.008, CHROME, 8)
  -- the shade: its outside bottom to top (Glenn: "the desk lamp's shade is inside out"), a lining the other way since
  -- it is open underneath, a lit bulb inside
  k.turned("hard", V(0, 0.34, 0.04), { { 0.07, 0.0 }, { 0.06, 0.04 }, { 0.02, 0.12 }, { 0.0, 0.12 } }, shade, 1, 16)
  k.turned("hard", V(0, 0.34, 0.04), { { 0.0, 0.115 }, { 0.017, 0.115 }, { 0.056, 0.038 }, { 0.066, 0.0 } }, shade * 0.6, 1, 16)
  k.turned("light", V(0, 0.36, 0.04), { { 0.0, 0.0 }, { 0.02, 0.01 }, { 0.022, 0.03 }, { 0.01, 0.05 }, { 0.0, 0.055 } },
           V(1.0, 0.92, 0.75), 1, 10)
  return k.done()
end

M.potted_plant = function(v)
  -- A HOUSEPLANT, GROWN (Glenn, 2026-10-03: "LOL is that a cactus on the teacher's desk. We should use the lsystem to
  -- build that."): a pot, and a plant from the engine's L-system -- the turtle's stems, a leaf card at every apex turned
  -- along its heading. The style bits pick the plant: a leafy bush (a ficus), an arching fern, a rosette of long upright
  -- leaves (a snake plant), a cactus; the variant seeds its growth.
  local k, style = maker(), styleOf(v)
  k.size = V(0.30, 0.55, 0.30)
  k.solid = false
  local pot = (v & 32) ~= 0 and WHITE or V(0.66, 0.36, 0.24)
  k.turned("ceramic", V(0, 0, 0.15), { { 0.0, 0.0 }, { 0.07, 0.0 }, { 0.095, 0.15 }, { 0.10, 0.16 }, { 0.0, 0.16 } }, pot, 1, 16)
  k.turned("fabric", V(0, 0.145, 0.15), { { 0.0, 0.0 }, { 0.092, 0.0 }, { 0.0, 0.012 } }, V(0.22, 0.15, 0.10), 1, 14)
  local species = (style >> 1) % 4
  local ls = lsystem.create()
  local tp = { radius_taper = species == 3 and 0.72 or 0.8, segment_slices = species == 3 and 12 or 5 }
  local axiom, iters, leafL, leafW = nil, 3, 0.07, 0.035
  if species == 0 then        -- bush: branching stems, leaves all over
    ls:rule("X", "F[+XL][-XL]&[XL]/F[^XL]", 1.0)
    ls:rule("X", "F[&XL][/+XL]F[-XL]", 1.0)
    axiom = "FX"; tp.length = 0.045; tp.radius = 0.006; tp.angle_deg = 32; iters = 3
  elseif species == 1 then    -- fern: fronds arching out, leaflets along them
    ls:rule("A", "F[+L][-L]^A", 1.0)
    axiom = "[&&A]/(72)[&&A]/(72)[&&A]/(72)[&&A]/(72)[&&A]"
    tp.length = 0.035; tp.radius = 0.003; tp.angle_deg = 22; iters = 6; leafL = 0.04; leafW = 0.016
  elseif species == 3 then    -- a CACTUS (Glenn: "that's not what a cactus looks like"): a ribbed column, arms that
                              -- turn out and then up, no leaves -- spines and a flower below
    axiom = (v & 64) ~= 0 and "FFF[&F^^FF]F/[&F^^F]F" or "FF[&F^^FFF]FF"
    tp.length = 0.06; tp.radius = 0.034; tp.angle_deg = 45; iters = 0
  else                        -- rosette: long upright blades from the soil
    axiom = "[&L]/[&L]/[&L]/[&L]/[&L]/[&L]/[L]"
    tp.length = 0.03; tp.radius = 0.004; tp.angle_deg = 14; iters = 0; leafL = 0.30; leafW = 0.05
  end
  local sym = iters > 0 and ls:expand(axiom, iters, seed(v * 2654435761 + 1)) or axiom
  sym = sym:gsub("[XA]", "L"):gsub("%b()", "")   -- the plain turtle has no "/(72)": read "/(n)" as one roll
  local soil = V(0, 0.155, 0.15)
  local stems = mesh.paint(lsystem.turtle_mesh(sym, tp), species == 3 and V(0.22, 0.42, 0.24) or V(0.25, 0.35, 0.15))
  if species == 3 then
    -- RIBS: a cactus is fluted -- every other vertex out along its normal (12 slices: every other one out) -- then
    -- spines down the ribs and a pink flower on the crown
    local pts, nrm = {}, {}
    stems = mesh.move_by(stems, function(i, p, n)
      local q = p
      if i % 2 == 0 then q = { p[1] + n[1] * 0.006, p[2] + n[2] * 0.006, p[3] + n[3] * 0.006 } end
      pts[i] = V(q[1], q[2], q[3]) + soil
      nrm[i] = V(n[1], n[2], n[3])
      return q
    end)
    local rr = rng.new(seed(v * 2246822519 + 9))
    for i = 0, #pts, 3 do
      if pts[i] and rr:next() <= 0.35 then
        k.rod("hard", pts[i], pts[i] + nrm[i] * 0.012, 0.0012, V(0.85, 0.82, 0.70), 3)
      end
    end
    local crown = V(0, 0, 0)
    for i = 0, #pts do
      if pts[i] and pts[i][2] > crown[2] then crown = pts[i] end
    end
    if (v & 128) ~= 0 then
      k.turned("fabric", crown + V(0, -0.005, 0), { { 0.0, 0.0 }, { 0.018, 0.012 }, { 0.012, 0.022 }, { 0.0, 0.016 } },
               V(0.90, 0.35, 0.55), 1, 8)
    end
  end
  k.put("fabric", mesh.translate(stems, soil))
  if species ~= 3 then
    local r = rng.new(seed(v * 747796405 + 3))
    local leafCol = species == 2 and V(0.20, 0.38, 0.16) or V(0.16, 0.40, 0.14)
    for _, lf in ipairs(lsystem.leaves(sym, tp)) do
      -- a leaf card: long along the heading, flat across it, a little colour each
      local d = V(lf.direction[1], lf.direction[2], lf.direction[3])
      if species == 2 then d = d + V(0, 1.4 * math.sqrt(d[1] * d[1] + d[2] * d[2] + d[3] * d[3]), 0) end
      local card = mesh.translate(mesh.rounded_box({ leafW, leafL, 0.004 }, 0.0, { segs = 1, uv = 2 }), { 0, leafL * 0.5, 0 })
      card = mesh.paint(mesh.orient(card, d), leafCol * (0.85 + 0.3 * r:next()))
      k.put("fabric", mesh.translate(card, V(lf.position[1], lf.position[2], lf.position[3]) + soil))
    end
  end
  return k.done()
end

M.book_stack = function(v)
  -- three or four books lying flat, each a little askew
  local k = maker()
  k.size = V(0.30, 0.16, 0.24)
  k.solid = false
  local covers = { V(0.55, 0.12, 0.10), V(0.12, 0.20, 0.45), V(0.15, 0.40, 0.22), V(0.85, 0.80, 0.70), V(0.10, 0.10, 0.10),
                   V(0.75, 0.55, 0.20) }
  local n = 3 + v % 2
  local y = 0
  for i = 0, n - 1 do
    local h, w, d = 0.028 + 0.012 * ((v + i) % 3), 0.24 - 0.02 * (i % 2), 0.17 + 0.015 * (i % 3)
    k.tilted("hard", V(0.01 * (((i * 3 + v) % 5) - 2), y + h * 0.5, 0.12), V(w, h, d), 0.003, 0.0, covers[(v // 2 + i) % 6 + 1])
    y = y + h
  end
  return k.done()
end

M.kettle = function(v)
  local k = maker()
  k.size = V(0.24, 0.26, 0.20)
  k.solid = false
  local body = (v & 32) ~= 0 and WHITE or STEEL
  k.turned("metal", V(0, 0, 0.10), { { 0.0, 0.0 }, { 0.085, 0.0 }, { 0.09, 0.03 }, { 0.08, 0.18 }, { 0.05, 0.21 }, { 0.0, 0.215 } },
           body, 1, 20)
  k.box("hard", V(0.10, 0.12, 0.10), V(0.025, 0.16, 0.03), 0.01, BLACK, 1)   -- the handle
  k.rod("metal", V(-0.07, 0.10, 0.10), V(-0.12, 0.17, 0.10), 0.012, body, 8) -- the spout
  return k.done()
end

M.fruit_bowl = function(v)
  local k = maker()
  k.size = V(0.30, 0.14, 0.30)
  k.solid = false
  k.turned("ceramic", V(0, 0, 0.15), { { 0.0, 0.0 }, { 0.06, 0.0 }, { 0.14, 0.07 }, { 0.145, 0.075 }, { 0.0, 0.03 } }, PORCELAIN, 1, 20)
  local fruit = { V(0.85, 0.15, 0.10), V(0.95, 0.70, 0.10), V(0.40, 0.65, 0.15), V(0.90, 0.45, 0.10) }
  for i = 0, 4 do
    local a, r = i * 1.257, i == 4 and 0.0 or 0.06
    k.turned("hard", V(r * math.cos(a), 0.05 + (i == 4 and 0.04 or 0.0), 0.15 + r * math.sin(a)),
             { { 0.0, 0.0 }, { 0.03, 0.01 }, { 0.038, 0.035 }, { 0.03, 0.06 }, { 0.0, 0.07 } }, fruit[(v + i) % 4 + 1], 1, 10)
  end
  return k.done()
end

M.vase = function(v)
  local k = maker()
  k.size = V(0.16, 0.48, 0.16)
  k.solid = false
  local glaze = { V(0.20, 0.35, 0.55), V(0.85, 0.82, 0.75), V(0.55, 0.25, 0.20) }
  k.turned("ceramic", V(0, 0, 0.08), { { 0.0, 0.0 }, { 0.05, 0.0 }, { 0.075, 0.10 }, { 0.05, 0.22 }, { 0.035, 0.26 }, { 0.045, 0.28 },
                                       { 0.0, 0.28 } }, glaze[v % 3 + 1], 1, 18)
  for i = 0, 4 do   -- stems
    k.rod("wood", V(0.0, 0.26, 0.08), V(0.04 * math.cos(i * 1.3), 0.42 + 0.04 * (i % 2), 0.08 + 0.04 * math.sin(i * 1.3)), 0.004,
          V(0.25, 0.40, 0.18), 6)
  end
  return k.done()
end

M.mug = function(v)
  local k = maker()
  k.size = V(0.12, 0.10, 0.09)
  k.solid = false
  local glaze = ({ V(0.95, 0.95, 0.93), V(0.20, 0.30, 0.55), V(0.75, 0.20, 0.18), V(0.20, 0.20, 0.22) })[v % 4 + 1]
  k.turned("ceramic", V(0, 0, 0.045), { { 0.0, 0.0 }, { 0.04, 0.0 }, { 0.042, 0.095 }, { 0.0, 0.095 } }, glaze, 1, 16)
  k.box("ceramic", V(0.05, 0.05, 0.045), V(0.015, 0.06, 0.012), 0.005, glaze, 1)
  return k.done()
end

-- THE UNIVERSITY CAMPUS: a dorm's single bed, a classroom's desk and stacking chair, a lecture hall's row of tip-up
-- seats on its own riser, the lectern, a lab's bench, the library's long reading table.
M.single_bed = function(v)
  -- A made single, 0.95 x 2.05: the frame on legs, a low timber headboard, the mattress, the duvet, one pillow.
  local k, wood, fabric = maker(), woodOf(v), fabricOf(v)
  k.size = V(0.95, 0.85, 2.05)
  k.box("wood", V(0, 0.22, 1.02), V(0.95, 0.12, 2.03), 0.01, wood, 1)
  for _, x in ipairs({ -0.42, 0.42 }) do
    for _, z in ipairs({ 0.06, 1.98 }) do k.turned_leg("wood", V(x, 0, z), 0.16, 0.026, 0.018, wood * 0.9) end
  end
  local hb = mesh.extrude(mesh.profile.rounded_rect(0.95, 0.05, 0.02, 3), 0.66, { bevel = 0.012, bevel_segs = 2, color = wood * 0.95 })
  k.put("wood", mesh.translate(hb, V(0, 0.19, 0.03)))      -- the headboard, its top rounded
  makeBed(k, 0.88, 0.48, 0.07, 1.99, 0.62, fabric, { 0 }, 0.30)
  return k.done()
end

M.school_desk = function(v)
  -- A pupil's desk: a laminate top on a tubular steel frame, a book shelf under it. (Glenn: "the parts of the students
  -- desks don't seem connected together": a welded frame -- legs into an apron, the shelf on rails.)
  local k, wood = maker(), woodOf(v)
  k.size = V(0.70, 0.74, 0.50)
  local steel = STEEL * 0.6
  k.box("hard", V(0, 0.725, 0.25), V(0.70, 0.025, 0.50), 0.006, wood * 1.1, 1)
  for _, x in ipairs({ -0.32, 0.32 }) do
    for _, z in ipairs({ 0.04, 0.46 }) do k.rod("metal", V(x, 0.0, z), V(x, 0.715, z), 0.013, steel, 8) end
  end
  for _, z in ipairs({ 0.04, 0.46 }) do k.rod("metal", V(-0.32, 0.70, z), V(0.32, 0.70, z), 0.012, steel, 8) end   -- apron
  for _, x in ipairs({ -0.32, 0.32 }) do
    k.rod("metal", V(x, 0.70, 0.04), V(x, 0.70, 0.46), 0.012, steel, 8)
    k.rod("metal", V(x, 0.55, 0.04), V(x, 0.55, 0.46), 0.010, steel, 8)   -- the shelf's rails
  end
  k.box("metal", V(0, 0.56, 0.25), V(0.64, 0.012, 0.42), 0.0, steel, 1)
  return k.done()
end

M.school_chair = function(v)
  -- A stacking chair: a moulded shell on four splayed steel legs.
  local k = maker()
  k.size = V(0.46, 0.80, 0.48)
  k.solid = false
  local shell = FABRIC[((v + 3) & 7) + 1] * 0.9
  k.box("hard", V(0, 0.45, 0.26), V(0.42, 0.03, 0.40), 0.015, shell)
  k.tilted("hard", V(0, 0.66, 0.05), V(0.40, 0.34, 0.025), 0.012, -0.12, shell)
  for _, x in ipairs({ -0.18, 0.18 }) do
    k.rod("metal", V(x * 1.1, 0.0, 0.44), V(x, 0.44, 0.40), 0.011, STEEL * 0.7, 8)
    k.rod("metal", V(x * 1.1, 0.0, 0.06), V(x, 0.44, 0.10), 0.011, STEEL * 0.7, 8)
    k.rod("metal", V(x, 0.44, 0.10), V(x, 0.62, 0.06), 0.011, STEEL * 0.7, 8)
  end
  return k.done()
end

M.lecture_row = function(v)
  -- A LECTURE HALL's row: six tip-up seats, the writing ledge for the row behind fixed to their backs, on its own
  -- carpeted RISER: style bits = the row's TIER, 0.18 m a tier -- a hall's rows step up toward the back, each standing on
  -- the flat floor as a solid block (the room program's grid numbers them).
  local k, wood, fabric = maker(), woodOf(v), fabricOf(v)
  local h = 0.18 * (styleOf(v) & 7)
  k.size = V(3.4, 1.1 + h, 1.0)
  -- the collider is the RISER, not the seats' box: you step onto your row from the aisle stair beside it (a
  -- full-height box made every row a wall)
  k.solid = h > 0
  k.collider_h = h
  local carpet = FABRIC[((v + 5) & 7) + 1] * 0.55
  if h > 0 then k.box("fabric", V(0, h * 0.5, 0.50), V(3.4, h, 1.0), 0.0, carpet, 1) end
  -- (a seat faces +z like every seat in the kit -- its back toward the riser's rear edge; Glenn: "those seats are
  -- facing the wrong way")
  for i = 0, 5 do
    local x = -1.4 + i * 0.56
    k.box("fabric", V(x, h + 0.44, 0.40), V(0.50, 0.08, 0.44), 0.025, fabric)                -- the seat
    k.tilted("fabric", V(x, h + 0.78, 0.16), V(0.50, 0.56, 0.07), 0.025, -0.12, fabric)      -- its back
    k.box("hard", V(x - 0.28, h + 0.35, 0.37), V(0.05, 0.62, 0.55), 0.01, BLACK * 2.5, 1)     -- the standard
  end
  k.box("hard", V(1.68, h + 0.35, 0.37), V(0.05, 0.62, 0.55), 0.01, BLACK * 2.5, 1)
  -- the writing ledge for the row behind, on this row's back
  k.box("wood", V(0, h + 0.74, 0.06), V(3.36, 0.03, 0.12), 0.005, wood, 1)
  k.box("hard", V(0, h + 0.40, 0.11), V(3.36, 0.70, 0.03), 0.005, BLACK * 2.5, 1)   -- the modesty panel
  return k.done()
end

M.lectern = function(v)
  local k, wood = maker(), woodOf(v)
  k.size = V(0.70, 1.15, 0.55)
  k.box("wood", V(0, 0.50, 0.30), V(0.60, 1.0, 0.45), 0.01, wood, 1)
  k.tilted("wood", V(0, 1.09, 0.28), V(0.70, 0.04, 0.52), 0.008, -0.25, wood * 1.05)
  k.box("wood", V(0, 0.03, 0.30), V(0.66, 0.06, 0.50), 0.004, wood * 0.75, 1)
  return k.done()
end

M.lab_bench = function(v)
  -- A LAB BENCH, 2.4 x 0.75: a black resin top over white cupboards, a sink at one end with its gooseneck tap, a
  -- service spine of sockets and gas taps along the back.
  local k = maker()
  k.size = V(2.4, 1.15, 0.75)
  k.box("hard", V(0, 0.44, 0.38), V(2.36, 0.80, 0.70), 0.006, WHITE, 1)
  for d = 0, 3 do
    local x0 = -1.16 + d * 0.58
    k.front("hard", x0, x0 + 0.58, 0.08, 0.80, 0.73, WHITE * 0.97, CHROME, true)
  end
  k.box("hard", V(0, 0.865, 0.375), V(2.4, 0.03, 0.75), 0.004, V(0.08, 0.08, 0.09), 1)
  k.box("ceramic", V(0.85, 0.86, 0.40), V(0.40, 0.03, 0.34), 0.01, V(0.12, 0.12, 0.13), 1)   -- the sink
  k.rod("metal", V(0.85, 0.88, 0.12), V(0.85, 1.12, 0.12), 0.012, CHROME, 8)
  k.rod("metal", V(0.85, 1.12, 0.12), V(0.85, 1.10, 0.30), 0.012, CHROME, 8)
  k.box("hard", V(-0.2, 1.0, 0.06), V(1.8, 0.24, 0.10), 0.01, STEEL * 0.85, 1)               -- the spine
  for i = 0, 4 do k.box("hard", V(-1.0 + i * 0.4, 1.0, 0.115), V(0.08, 0.06, 0.01), 0.0, WHITE, 1) end
  return k.done()
end

M.reading_table = function(v)
  -- The LIBRARY's long table for eight: an oak top, a green-shaded lamp at each end, four chairs a side (part of the
  -- piece: a reading room is tables in rows).
  local k, wood = maker(), woodOf(v)
  k.size = V(2.6, 1.16, 2.0)
  k.box("wood", V(0, 0.74, 1.0), V(2.4, 0.045, 0.95), 0.01, wood, 1)
  for _, x in ipairs({ -1.1, 1.1 }) do
    for _, z in ipairs({ 0.6, 1.4 }) do k.leg("wood", V(x, 0, z), 0.72, 0.07, wood * 0.9) end
  end
  -- the BANKER'S LAMPS (Glenn: "the green part is a shell and there's a lightbulb or two inside"): an open half-tube of
  -- green glass on a brass stem, two lit bulbs under it
  local brass, green = V(0.55, 0.45, 0.25), V(0.10, 0.34, 0.18)
  for _, x in ipairs({ -0.95, 0.95 }) do
    k.turned("metal", V(x, 0.765, 1.0), { { 0.07, 0.0 }, { 0.07, 0.015 }, { 0.012, 0.03 }, { 0.012, 0.30 }, { 0.0, 0.31 } }, brass, 1, 12)
    k.rod("metal", V(x, 1.07, 1.0), V(x, 1.13, 1.0), 0.008, brass, 6)
    for seg = 0, 6 do   -- the shade: a half tube along x, open underneath
      local a0, a1 = PI * seg / 7.0, PI * (seg + 1) / 7.0
      local am = (a0 + a1) * 0.5
      local r = 0.075
      local cw = r * (a1 - a0) + 0.004
      k.tilted("ceramic", V(x, 1.10 + r * math.sin(am), 1.0 + r * math.cos(am)), V(0.34, 0.008, cw), 0.0, PI * 0.5 - am, green)
    end
    for _, bx in ipairs({ -0.07, 0.07 }) do
      k.turned("light", V(x + bx, 1.085, 1.0), { { 0.0, 0.0 }, { 0.022, 0.012 }, { 0.024, 0.03 }, { 0.012, 0.05 }, { 0.0, 0.055 } },
               V(1.0, 0.92, 0.75), 1, 10)
    end
  end
  for _, x in ipairs({ -0.9, -0.3, 0.3, 0.9 }) do
    for _, s2 in ipairs({ -1.0, 1.0 }) do
      local zc = 1.0 + s2 * 0.72
      -- (Glenn: "the wooden chairs have a floating head rest": the rear legs run up past the seat and carry the back)
      k.box("wood", V(x, 0.46, zc), V(0.44, 0.05, 0.42), 0.012, wood * 0.95, 1)
      for _, lx in ipairs({ -0.18, 0.18 }) do
        k.leg("wood", V(x + lx, 0, zc - s2 * 0.17), 0.44, 0.035, wood * 0.9)   -- front legs
        k.leg("wood", V(x + lx, 0, zc + s2 * 0.19), 0.95, 0.035, wood * 0.9)   -- rear legs, up the back
      end
      k.box("wood", V(x, 0.86, zc + s2 * 0.19), V(0.40, 0.16, 0.03), 0.008, wood * 0.95, 1)   -- top rail
      k.box("wood", V(x, 0.64, zc + s2 * 0.19), V(0.36, 0.05, 0.025), 0.006, wood * 0.95, 1)  -- mid rail
    end
  end
  return k.done()
end

M.aisle_step = function(v)
  -- THE AISLE STAIR (Glenn: "there should be stairs that allow you to access the raised auditorium chairs"): beside
  -- each row, the aisle climbs its tier in two 9 cm steps -- the lower at the row's front half, the tier's own height at
  -- its back half -- so the aisle rises with the rows, a step at a time.
  local k = maker()
  local h = 0.18 * (styleOf(v) & 7)
  k.size = V(0.9, math.max(h, 0.02), 1.0)
  k.solid = h > 0
  k.collider_h = math.max(h - 0.09, 0.0)   -- the walkable truth: the lower tread (the step is 9 cm)
  local carpet, nose = FABRIC[((v + 5) & 7) + 1] * 0.55, V(0.75, 0.70, 0.40)
  if h > 0 then
    local lo = math.max(h - 0.09, 0.0)
    if lo > 0 then k.box("fabric", V(0, lo * 0.5, 0.75), V(0.9, lo, 0.5), 0.0, carpet, 1) end
    k.box("fabric", V(0, h * 0.5, 0.25), V(0.9, h, 0.5), 0.0, carpet, 1)
    k.box("hard", V(0, h + 0.002, 0.49), V(0.9, 0.004, 0.03), 0.0, nose, 1)    -- nosing strips
    if lo > 0 then k.box("hard", V(0, lo + 0.002, 0.99), V(0.9, 0.004, 0.03), 0.0, nose, 1) end
  else
    k.box("fabric", V(0, 0.005, 0.5), V(0.9, 0.01, 1.0), 0.0, carpet, 1)   -- the front row's aisle: a runner
  end
  return k.done()
end

M.bleacher = function(v)
  -- A SPORTS FIELD'S STAND: three tiers of aluminium bench planks rising away from the pitch (+z faces it), footboards
  -- between, a galvanised frame of uprights and raked stringers, a rail along the back.
  local k = maker()
  k.size = V(6.0, 2.1, 2.6)
  k.solid = false
  local alu, galv = V(0.78, 0.80, 0.82), V(0.55, 0.57, 0.60)
  for t = 0, 2 do
    local seatY, z = 0.45 + 0.42 * t, 2.05 - 0.72 * t
    k.box("metal", V(0, seatY, z), V(6.0, 0.05, 0.28), 0.008, alu, 1)                        -- the seat plank
    k.box("metal", V(0, seatY - 0.40, z + 0.30), V(6.0, 0.04, 0.32), 0.006, alu * 0.92, 1)   -- the footboard
  end
  for _, x in ipairs({ -2.85, -0.95, 0.95, 2.85 }) do
    k.rod("metal", V(x, 0.0, 2.25), V(x, 0.45, 2.25), 0.03, galv, 8)    -- front leg
    k.rod("metal", V(x, 0.0, 0.40), V(x, 1.95, 0.40), 0.03, galv, 8)    -- rear leg
    k.rod("metal", V(x, 0.43, 2.25), V(x, 1.30, 0.40), 0.025, galv, 8)  -- raked stringer
  end
  k.rod("metal", V(-3.0, 1.95, 0.40), V(3.0, 1.95, 0.40), 0.025, galv, 8)   -- the back rail
  return k.done()
end

-- --- RESTAURANTS, BARS AND CLUBS (storefronts stage 5; Glenn: "What about the interior does it look like a shop would
-- look?"). Every one faces +z, its back on z = 0.
M.bar_counter = function(v)
  -- A BAR: a 1.8 m run of the counter -- a panelled front with a brass foot rail, a thick top overhanging the
  -- customers' side, the speed rail and a drip mat on the bartender's. Runs butt end to end.
  local k = maker()
  k.size = V(1.8, 1.12, 0.75)
  k.collider_h = 1.08
  local wood = barWood(v)   -- (a bar is walnut or ebonised, never birch)
  local top, brass = (styleOf(v) & 1) ~= 0 and V(0.12, 0.08, 0.06) or wood * 0.8, V(0.62, 0.48, 0.22)
  k.box("wood", V(0, 0.53, 0.30), V(1.8, 1.06, 0.50), 0.006, wood * 0.75, 1)   -- the carcass
  for i = 0, 2 do                                                              -- raised panels, customers' side
    k.box("wood", V(-0.6 + i * 0.6, 0.55, 0.56), V(0.50, 0.70, 0.03), 0.008, wood * 0.9, 1)
  end
  k.box("wood", V(0, 1.085, 0.38), V(1.82, 0.06, 0.74), 0.012, top, 2)         -- the top
  k.rod("metal", V(-0.9, 0.22, 0.68), V(0.9, 0.22, 0.68), 0.025, brass, 10)    -- the foot rail
  for _, x in ipairs({ -0.6, 0.6 }) do k.rod("metal", V(x, 0.08, 0.58), V(x, 0.22, 0.68), 0.012, brass, 8) end
  k.box("hard", V(0.3, 1.12, 0.12), V(0.6, 0.012, 0.16), 0.0, BLACK * 2.0, 1)  -- the drip mat
  return k.done()
end

M.back_bar = function(v)
  -- The BACK BAR: low cupboards and a fridge door, a mirror, two lit glass shelves of bottles.
  local k = maker()
  k.size = V(1.8, 2.3, 0.5)
  k.collider_h = 0.9
  local wood = barWood(v)
  k.box("wood", V(0, 0.45, 0.25), V(1.8, 0.9, 0.48), 0.006, wood * 0.7, 1)
  k.front("hard", -0.88, -0.02, 0.06, 0.86, 0.49, V(0.10, 0.11, 0.12), CHROME, true)   -- the bottle fridge
  k.front("wood", 0.02, 0.88, 0.06, 0.86, 0.49, wood * 0.8, CHROME, true)
  k.box("hard", V(0, 0.92, 0.25), V(1.82, 0.04, 0.50), 0.004, BLACK * 2.0, 1)
  k.box("metal", V(0, 1.62, 0.02), V(1.7, 1.3, 0.02), 0.0, V(0.55, 0.60, 0.64), 1)       -- the mirror
  local r = rng.new(seed(v * 2654435761 + 29))
  local bottles = { V(0.45, 0.25, 0.08), V(0.15, 0.35, 0.18), V(0.85, 0.85, 0.80), V(0.55, 0.12, 0.10), V(0.75, 0.55, 0.20),
                    V(0.20, 0.25, 0.45) }
  for _, y in ipairs({ 1.30, 1.75 }) do
    k.box("metal", V(0, y, 0.15), V(1.7, 0.015, 0.26), 0.0, V(0.70, 0.78, 0.80), 1)        -- the glass shelf
    k.box("light", V(0, y - 0.015, 0.05), V(1.6, 0.01, 0.02), 0.0, V(1.0, 0.75, 0.45), 1)  -- its strip
    for i = 0, 11 do
      local h = 0.22 + 0.12 * r:next()
      k.turned("hard", V(-0.77 + i * 0.14, y + 0.008, 0.15),
               { { 0.0, 0.0 }, { 0.04, 0.0 }, { 0.04, h * 0.65 }, { 0.014, h * 0.8 }, { 0.014, h }, { 0.0, h } }, pick(bottles, r:next()), 1, 8)
    end
  end
  return k.done()
end

M.bar_stool = function(v)
  -- A BAR STOOL: a round seat at 0.76 on a chrome column, a foot ring, a weighted base.
  local k = maker()
  k.size = V(0.42, 0.80, 0.42)
  k.solid = false
  k.turned("metal", V(0, 0, 0.21), { { 0.0, 0.0 }, { 0.19, 0.0 }, { 0.19, 0.02 }, { 0.04, 0.04 }, { 0.0, 0.04 } }, CHROME, 1, 16)
  k.rod("metal", V(0, 0.03, 0.21), V(0, 0.74, 0.21), 0.025, CHROME, 10)
  -- the foot ring: a hoop swept round the column
  k.put("metal", mesh.sweep(mesh.profile.circle(0.009, 6), mesh.path.circle({ 0, 0.30, 0.21 }, 0.16, { 0, 1, 0 }, 16),
                            { color = CHROME }))
  local leather = { V(0.10, 0.09, 0.09), V(0.32, 0.07, 0.06), V(0.45, 0.28, 0.15) }   -- black, oxblood, tan
  k.turned("fabric", V(0, 0.72, 0.21), { { 0.0, 0.0 }, { 0.19, 0.0 }, { 0.20, 0.04 }, { 0.17, 0.08 }, { 0.0, 0.08 } },
           leather[(v >> 3) % 3 + 1], 1, 18)
  return k.done()
end

M.booth = function(v)
  -- A BOOTH: two high-backed upholstered benches facing across a table, its end against the wall (+z: out).
  local k = maker()
  k.size = V(1.9, 1.15, 1.3)
  k.collider_h = 0.75
  local wood = barWood(v)
  local hide = (styleOf(v) & 1) ~= 0 and V(0.40, 0.08, 0.08) or V(0.12, 0.10, 0.10)   -- oxblood or black hide
  for _, s in ipairs({ -1.0, 1.0 }) do
    local x = s * 0.70
    k.box("wood", V(x, 0.22, 0.65), V(0.50, 0.44, 1.26), 0.006, wood * 0.7, 1)              -- the plinth
    k.box("fabric", V(x - s * 0.02, 0.50, 0.65), V(0.46, 0.12, 1.22), 0.04, hide, 3)         -- the seat
    k.box("fabric", V(s * 0.88, 0.80, 0.65), V(0.12, 0.70, 1.22), 0.04, hide * 0.92, 3)      -- the back
  end
  k.box("wood", V(0, 0.74, 0.60), V(0.76, 0.04, 1.10), 0.01, wood, 1)                        -- the table
  k.rod("metal", V(0, 0.0, 0.60), V(0, 0.72, 0.60), 0.04, BLACK, 10)
  k.box("metal", V(0, 0.01, 0.60), V(0.5, 0.02, 0.5), 0.004, BLACK, 1)
  k.turned("light", V(0, 0.76, 0.20), { { 0.0, 0.0 }, { 0.04, 0.0 }, { 0.035, 0.08 }, { 0.0, 0.10 } }, V(1.0, 0.70, 0.40), 1, 10)
  return k.done()
end

M.high_table = function(v)
  -- A POSEUR TABLE: a small round top at elbow height on a pedestal.
  local k, wood = maker(), woodOf(v)
  k.size = V(0.7, 1.08, 0.7)
  k.collider_h = 1.05
  k.turned("metal", V(0, 0, 0.35), { { 0.27, 0.0 }, { 0.27, 0.02 }, { 0.03, 0.05 }, { 0.03, 1.04 }, { 0.0, 1.05 } }, BLACK, 1, 14)
  k.turned("wood", V(0, 1.04, 0.35), { { 0.0, 0.0 }, { 0.33, 0.0 }, { 0.33, 0.035 }, { 0.0, 0.035 } }, wood * 0.8, 1, 22)
  return k.done()
end

M.dj_booth = function(v)
  -- The DJ BOOTH: a black riser and desk, two decks and a mixer, a stack of speakers at each end, a lit strip along its
  -- front (+z: the floor).
  local k = maker()
  k.size = V(3.0, 2.0, 1.0)
  k.collider_h = 1.05
  k.box("hard", V(0, 0.10, 0.5), V(3.0, 0.20, 1.0), 0.004, BLACK * 1.5, 1)                    -- the riser
  k.box("hard", V(0, 0.62, 0.72), V(1.8, 0.84, 0.10), 0.006, BLACK, 1)                        -- the desk front
  k.box("light", V(0, 0.40, 0.78), V(1.7, 0.03, 0.01), 0.0, V(0.20, 0.55, 1.0), 1)
  k.box("light", V(0, 0.80, 0.78), V(1.7, 0.03, 0.01), 0.0, V(0.95, 0.20, 0.75), 1)
  k.box("hard", V(0, 1.06, 0.55), V(1.8, 0.04, 0.45), 0.004, BLACK * 2.0, 1)                  -- the desk top
  for _, x in ipairs({ -0.55, 0.55 }) do
    k.box("hard", V(x, 1.11, 0.55), V(0.46, 0.06, 0.36), 0.006, STEEL * 0.5, 1)
    k.turned("hard", V(x - 0.04, 1.14, 0.55), { { 0.0, 0.0 }, { 0.15, 0.0 }, { 0.15, 0.012 }, { 0.0, 0.012 } }, BLACK, 1, 20)
  end
  k.box("hard", V(0, 1.12, 0.55), V(0.30, 0.08, 0.34), 0.006, STEEL * 0.4, 1)                  -- the mixer
  for _, x in ipairs({ -1.25, 1.25 }) do
    for t = 0, 1 do
      local y0 = 0.20 + t * 0.88
      k.box("hard", V(x, y0 + 0.43, 0.45), V(0.5, 0.86, 0.6), 0.008, BLACK * 1.2, 1)
      k.box("hard", V(x, y0 + 0.36, 0.755), V(0.34, 0.34, 0.012), 0.0, V(0.03, 0.03, 0.03), 1)   -- woofer
      k.box("hard", V(x, y0 + 0.36, 0.762), V(0.12, 0.12, 0.012), 0.0, STEEL * 0.3, 1)           -- its cap
      k.box("hard", V(x, y0 + 0.72, 0.755), V(0.12, 0.08, 0.012), 0.0, V(0.03, 0.03, 0.03), 1)    -- horn
    end
  end
  return k.done()
end

M.dance_floor = function(v)
  -- A LIT DANCE FLOOR: a 4 x 4 m grid of glass tiles, each lit in one of the club's colours, in a dark frame.
  local k = maker()
  k.size = V(4.0, 0.06, 4.0)
  k.solid = false
  local tiles = { V(1.0, 0.25, 0.70), V(0.25, 0.55, 1.0), V(0.95, 0.75, 0.25), V(0.30, 0.95, 0.70), V(0.65, 0.30, 1.0) }
  k.box("hard", V(0, 0.02, 2.0), V(4.0, 0.04, 4.0), 0.0, BLACK, 1)
  local r = rng.new(seed(v * 2246822519 + 17))
  for i = 0, 7 do
    for j = 0, 7 do
      local lit = ((i + j) & 1) == 0 or r:next() < 0.3
      local c = pick(tiles, r:next()) * (lit and 0.9 or 0.12)
      k.box(lit and "light" or "hard", V(-1.75 + i * 0.5, 0.045, 0.25 + j * 0.5), V(0.46, 0.01, 0.46), 0.0, c, 1)
    end
  end
  return k.done()
end

M.kitchen_pass = function(v)
  -- The PASS: a stainless counter between the kitchen and the dining room, a heat-lamp gantry over it, plates waiting
  -- under the lamps (+z: the dining room).
  local k = maker()
  k.size = V(2.4, 1.75, 0.8)
  k.collider_h = 0.95
  k.box("metal", V(0, 0.46, 0.40), V(2.4, 0.92, 0.76), 0.004, STEEL, 1)
  k.box("metal", V(0, 0.935, 0.40), V(2.44, 0.03, 0.80), 0.004, CHROME, 1)
  for _, x in ipairs({ -1.1, 1.1 }) do k.rod("metal", V(x, 0.95, 0.40), V(x, 1.70, 0.40), 0.02, CHROME, 8) end
  k.box("metal", V(0, 1.70, 0.40), V(2.3, 0.06, 0.30), 0.004, STEEL * 0.8, 1)                  -- the gantry
  k.box("light", V(0, 1.665, 0.40), V(2.1, 0.01, 0.12), 0.0, V(1.0, 0.55, 0.25), 1)           -- its lamps
  for i = 0, 4 do
    k.turned("ceramic", V(-0.8 + i * 0.4, 0.95, 0.40), { { 0.0, 0.0 }, { 0.10, 0.0 }, { 0.14, 0.02 }, { 0.0, 0.02 } }, PORCELAIN, 1, 16)
  end
  return k.done()
end

M.desk_screen = function(v)
  -- A DESK SCREEN (Glenn, 2026-10-13: "no office partitions in the office?"): a fabric panel between two desks in a
  -- row, running out from the wall (+z) past the desk and the chair, on two steel feet. 1.12 m tall -- desk height and
  -- a hand's span over it, so it stands under a window too (furniture.cpp's window rule).
  local k, fabric = maker(), fabricOf(v)
  k.size = V(0.05, 1.12, 1.30)
  k.collider_h = 1.12
  k.box("fabric", V(0, 0.61, 0.65), V(0.04, 1.0, 1.26), 0.012, fabric * 0.6 + grey(0.18))
  k.box("metal", V(0, 1.115, 0.65), V(0.05, 0.01, 1.28), 0.0, STEEL, 1)              -- the cap rail
  for _, z in ipairs({ 0.12, 1.18 }) do
    k.box("metal", V(0, 0.06, z), V(0.03, 0.12, 0.03), 0.0, STEEL, 1)                -- a leg
    k.box("metal", V(0, 0.008, z), V(0.05, 0.016, 0.24), 0.0, STEEL * 0.85, 1)       -- its foot
  end
  return k.done()
end

M.reception_desk = function(v)
  -- A RECEPTION DESK, 2.4 m: the worktop at desk height on the staff's side (-z, toward the screen behind), a panelled
  -- front standing to 1.08 m on the visitors' side (+z) with a ledge on top to sign in at, the monitor turned to the
  -- staff.
  local k, wood = maker(), woodOf(v)
  k.size = V(2.4, 1.10, 0.80)
  k.box("wood", V(0, 0.725, 0.32), V(2.36, 0.03, 0.60), 0.006, wood, 1)              -- the worktop
  k.box("hard", V(0, 0.54, 0.74), V(2.40, 1.08, 0.08), 0.01, WHITE, 1)               -- the front
  for _, x in ipairs({ -1.16, 1.16 }) do k.box("hard", V(x, 0.54, 0.40), V(0.08, 1.08, 0.72), 0.01, WHITE, 1) end   -- ends
  k.box("wood", V(0, 1.085, 0.66), V(2.44, 0.03, 0.28), 0.006, wood * 1.05, 1)       -- the ledge
  k.box("hard", V(0.75, 0.33, 0.30), V(0.42, 0.62, 0.50), 0.004, STEEL * 0.85, 1)    -- a pedestal
  k.box("hard", V(-0.4, 0.76, 0.42), V(0.22, 0.012, 0.16), 0.004, BLACK, 1)          -- the monitor's foot
  k.box("hard", V(-0.4, 0.92, 0.44), V(0.04, 0.30, 0.02), 0.004, BLACK, 1)
  k.box("hard", V(-0.4, 1.02, 0.41), V(0.58, 0.32, 0.022), 0.004, BLACK, 1)          -- its screen, facing -z
  return k.done()
end

M.hospital_bed = function(v)
  -- A HOSPITAL BED (Glenn, 2026-10-13: "do hospitals"): a steel frame on four castors, the mattress at 0.6 m with its
  -- head section raised, a sheet and a pillow, a plastic head- and footboard, folding side rails. The head at z = 0.
  local k, fabric = maker(), fabricOf(v)
  k.size = V(1.00, 1.00, 2.20)
  for _, x in ipairs({ -0.40, 0.40 }) do
    for _, z in ipairs({ 0.15, 2.05 }) do
      k.rod("metal", V(x, 0.10, z), V(x, 0.42, z), 0.025, STEEL, 8)
      k.turned("hard", V(x, 0, z), { { 0.0, 0.0 }, { 0.05, 0.0 }, { 0.05, 0.10 }, { 0.0, 0.10 } }, BLACK, 1, 10)   -- castor
    end
  end
  k.box("metal", V(0, 0.44, 1.10), V(0.90, 0.06, 2.00), 0.01, STEEL, 1)              -- the frame
  k.box("fabric", V(0, 0.55, 1.35), V(0.88, 0.16, 1.40), 0.04, LINEN)                -- the mattress, flat part
  k.tilted("fabric", V(0, 0.68, 0.40), V(0.88, 0.14, 0.72), 0.04, 0.45, LINEN)       -- its raised head section
  k.box("fabric", V(0, 0.84, 0.24), V(0.60, 0.10, 0.32), 0.04, LINEN * 1.03)         -- the pillow
  k.box("fabric", V(0, 0.64, 1.50), V(0.92, 0.04, 1.10), 0.02, fabric * 0.9 + grey(0.1))   -- blanket
  k.box("hard", V(0, 0.66, 0.03), V(0.98, 0.62, 0.05), 0.02, WHITE)                  -- headboard
  k.box("hard", V(0, 0.58, 2.17), V(0.98, 0.42, 0.05), 0.02, WHITE)                  -- footboard
  for _, x in ipairs({ -0.47, 0.47 }) do
    k.box("metal", V(x, 0.78, 0.75), V(0.03, 0.03, 0.80), 0.0, CHROME, 1)            -- the side rails
    k.box("metal", V(x, 0.62, 0.75), V(0.02, 0.32, 0.02), 0.0, CHROME, 1)
  end
  return k.done()
end

M.exam_couch = function(v)
  -- An EXAMINATION COUCH: a padded top on a steel base, the head end raised, a paper roll across it.
  local k = maker()
  k.size = V(0.70, 0.90, 1.90)
  local vinyl = V(0.30, 0.45, 0.55)
  k.box("metal", V(0, 0.30, 0.95), V(0.50, 0.60, 1.40), 0.01, STEEL * 0.85, 1)
  k.box("fabric", V(0, 0.66, 1.20), V(0.68, 0.10, 1.36), 0.03, vinyl)
  k.tilted("fabric", V(0, 0.76, 0.30), V(0.68, 0.10, 0.56), 0.03, 0.35, vinyl)
  k.box("fabric", V(0, 0.72, 1.20), V(0.50, 0.01, 1.30), 0.0, LINEN, 1)              -- the paper
  return k.done()
end

M.waiting_seats = function(v)
  -- WAITING SEATS: four moulded seats on one steel beam, two legs.
  local k, fabric = maker(), fabricOf(v)
  k.size = V(2.20, 0.82, 0.62)
  k.box("metal", V(0, 0.38, 0.32), V(2.10, 0.05, 0.06), 0.0, STEEL, 1)               -- the beam
  for _, x in ipairs({ -0.85, 0.85 }) do
    k.box("metal", V(x, 0.19, 0.32), V(0.05, 0.38, 0.05), 0.0, STEEL, 1)
    k.box("metal", V(x, 0.01, 0.32), V(0.06, 0.02, 0.56), 0.0, STEEL, 1)             -- the foot
  end
  for i = 0, 3 do
    local x = -0.79 + i * 0.527
    k.box("hard", V(x, 0.45, 0.34), V(0.48, 0.05, 0.46), 0.02, fabric * 0.8)                -- the seat
    k.tilted("hard", V(x, 0.66, 0.08), V(0.48, 0.40, 0.04), 0.02, -0.12, fabric * 0.8)      -- its back
  end
  return k.done()
end

M.pew = function(v)
  -- A PEW (Glenn, 2026-10-13: the church was empty): a 3 m timber bench, its back raked, carved ends, the kneeler on the
  -- back for the row behind. Faces +z (the altar).
  local k, wood = maker(), woodOf(v)
  k.size = V(3.0, 0.95, 0.75)
  k.box("wood", V(0, 0.44, 0.42), V(2.92, 0.05, 0.42), 0.01, wood, 1)                 -- the seat
  k.tilted("wood", V(0, 0.70, 0.17), V(2.92, 0.50, 0.04), 0.01, -0.14, wood)          -- the back
  for _, x in ipairs({ -1.47, 1.47 }) do k.box("wood", V(x, 0.47, 0.37), V(0.06, 0.94, 0.70), 0.02, wood * 0.85, 1) end   -- ends
  k.box("wood", V(0, 0.76, 0.06), V(2.92, 0.03, 0.10), 0.005, wood * 0.9, 1)          -- the book ledge behind
  k.box("fabric", V(0, 0.07, 0.04), V(2.80, 0.07, 0.12), 0.02, V(0.45, 0.12, 0.12))   -- the kneeler
  return k.done()
end

M.altar = function(v)
  -- The ALTAR: a table under a white cloth with a coloured frontal, two candlesticks, a book on its stand.
  local k, wood = maker(), woodOf(v)
  k.size = V(2.0, 1.35, 0.90)
  k.box("wood", V(0, 0.48, 0.45), V(1.96, 0.96, 0.86), 0.01, wood * 0.8, 1)
  k.box("fabric", V(0, 0.975, 0.45), V(2.0, 0.03, 0.90), 0.005, LINEN, 1)
  k.box("fabric", V(0, 0.70, 0.905), V(1.20, 0.52, 0.01), 0.0, V(0.50, 0.10, 0.14), 1)   -- the frontal
  for _, x in ipairs({ -0.75, 0.75 }) do   -- candlesticks
    k.turned("metal", V(x, 0.99, 0.30), { { 0.06, 0.0 }, { 0.02, 0.04 }, { 0.015, 0.28 }, { 0.035, 0.30 }, { 0.012, 0.31 },
                                         { 0.012, 0.36 }, { 0.0, 0.36 } }, V(0.80, 0.65, 0.30), 1, 10)
  end
  k.tilted("wood", V(0, 1.05, 0.45), V(0.36, 0.03, 0.26), 0.004, -0.3, wood)          -- the book stand
  return k.done()
end

M.cinema_row = function(v)
  -- A CINEMA ROW: six plush seats with arms and cup holders on a carpeted RISER -- style bits the row's tier, 0.25 m a
  -- tier (a cinema rakes steeper than a lecture hall). Faces +z, the screen.
  local k = maker()
  local h = 0.25 * (styleOf(v) & 7)
  k.size = V(3.4, 1.05 + h, 1.0)
  k.solid = h > 0
  k.collider_h = h
  local carpet, plush = V(0.18, 0.08, 0.10), V(0.55, 0.08, 0.10)
  if h > 0 then k.box("fabric", V(0, h * 0.5, 0.50), V(3.4, h, 1.0), 0.0, carpet, 1) end
  for i = 0, 5 do
    local x = -1.4 + i * 0.56
    k.box("fabric", V(x, h + 0.42, 0.42), V(0.50, 0.12, 0.48), 0.04, plush)                -- the seat
    k.tilted("fabric", V(x, h + 0.74, 0.14), V(0.50, 0.62, 0.12), 0.04, -0.15, plush)      -- its back
  end
  for i = 0, 6 do k.box("hard", V(-1.68 + i * 0.56, h + 0.55, 0.40), V(0.06, 0.20, 0.50), 0.02, BLACK * 2.0, 1) end   -- arms
  return k.done()
end

M.cinema_screen = function(v)
  -- The SCREEN: a 6 m white panel framed in black masking, hung high on the auditorium's front wall.
  local k = maker()
  k.size = V(6.0, 4.4, 0.08)
  k.solid = false
  k.box("hard", V(0, 2.70, 0.03), V(6.0, 3.4, 0.05), 0.0, BLACK, 1)
  k.box("hard", V(0, 2.70, 0.06), V(5.6, 3.0, 0.01), 0.0, V(0.92, 0.92, 0.94), 1)
  return k.done()
end

M.market_stall = function(v)
  -- A MARKET STALL: a trestle counter heaped with crates of produce, a striped awning on four posts.
  local k, wood = maker(), woodOf(v)
  k.size = V(2.4, 2.45, 1.4)
  local stripe = FABRIC[((v + 2) & 7) + 1]
  k.box("wood", V(0, 0.82, 0.95), V(2.3, 0.05, 0.85), 0.01, wood, 1)                  -- the counter
  k.box("wood", V(0, 0.42, 1.36), V(2.3, 0.78, 0.04), 0.01, wood * 0.85, 1)           -- its front
  local produce = { V(0.75, 0.15, 0.10), V(0.90, 0.55, 0.10), V(0.35, 0.60, 0.20), V(0.85, 0.80, 0.25) }
  for i = 0, 3 do
    local x = -0.82 + i * 0.55
    k.box("wood", V(x, 0.91, 1.0), V(0.48, 0.13, 0.60), 0.005, wood * 1.1, 1)       -- a crate
    k.box("hard", V(x, 0.99, 1.0), V(0.42, 0.06, 0.52), 0.03, produce[((i + v) & 3) + 1])   -- its fruit
  end
  for _, x in ipairs({ -1.14, 1.14 }) do
    for _, z in ipairs({ 0.06, 1.34 }) do k.rod("metal", V(x, 0, z), V(x, 2.25, z), 0.025, STEEL, 6) end
  end
  for i = 0, 5 do   -- the awning
    k.tilted("fabric", V(-1.0 + i * 0.4, 2.32, 0.70), V(0.40, 0.03, 1.46), 0.0, 0.08, (i & 1) ~= 0 and stripe or LINEN)
  end
  return k.done()
end

M.judge_bench = function(v)
  -- The BENCH: the judge's raised desk across the courtroom's front, panelled, the judge's high-backed chair behind it
  -- (at -z, toward the wall); the witness box at one end, the clerk's lower desk at the other.
  local k, wood = maker(), woodOf(v)
  k.size = V(4.0, 1.85, 2.2)
  local red = V(0.40, 0.12, 0.12)
  k.box("wood", V(0, 0.15, 0.80), V(2.6, 0.30, 1.6), 0.0, wood * 0.75, 1)              -- the dais
  k.box("wood", V(0, 0.70, 1.48), V(2.6, 1.10, 0.10), 0.01, wood, 1)                   -- the bench front
  k.box("wood", V(0, 1.24, 1.33), V(2.6, 0.04, 0.40), 0.005, wood * 1.1, 1)            -- its top
  k.box("fabric", V(0, 0.80, 0.55), V(0.60, 0.10, 0.55), 0.03, red)                    -- the chair's seat
  k.box("fabric", V(0, 1.30, 0.30), V(0.62, 1.00, 0.10), 0.03, red)                    -- its high back
  -- the witness box (right) and the clerk's desk (left)
  k.box("wood", V(1.65, 0.55, 1.20), V(0.70, 1.10, 0.06), 0.01, wood, 1)
  k.box("wood", V(1.98, 0.55, 0.85), V(0.06, 1.10, 0.70), 0.01, wood, 1)
  k.box("wood", V(1.65, 0.08, 0.85), V(0.70, 0.16, 0.70), 0.0, wood * 0.75, 1)
  k.box("wood", V(-1.65, 0.37, 1.70), V(0.70, 0.74, 0.50), 0.01, wood, 1)
  return k.done()
end

M.jury_box = function(v)
  -- The JURY BOX: two rows of six chairs behind a panelled rail, the back row on a step. Faces +z (the courtroom), to be
  -- set against a side wall.
  local k, wood, fabric = maker(), woodOf(v), fabricOf(v)
  k.size = V(3.6, 1.55, 2.0)
  k.box("wood", V(0, 0.10, 0.55), V(3.6, 0.20, 1.10), 0.0, wood * 0.75, 1)             -- the back row's step
  k.box("wood", V(0, 0.50, 1.96), V(3.6, 1.0, 0.08), 0.01, wood, 1)                    -- the front rail
  for _, x in ipairs({ -1.78, 1.78 }) do k.box("wood", V(x, 0.50, 1.0), V(0.06, 1.0, 1.92), 0.01, wood, 1) end
  for row = 0, 1 do
    for i = 0, 5 do
      local x, z, y0 = -1.45 + i * 0.58, row == 0 and 0.55 or 1.45, row == 0 and 0.20 or 0.0
      k.box("fabric", V(x, y0 + 0.46, z), V(0.46, 0.07, 0.44), 0.025, fabric * 0.8)
      k.box("fabric", V(x, y0 + 0.80, z - 0.21), V(0.44, 0.55, 0.05), 0.025, fabric * 0.8)
    end
  end
  return k.done()
end

M.gear_locker = function(v)
  -- A GEAR LOCKER: an open steel rack, a firefighter's turnout coat on its hook, the trousers folded down over the
  -- boots below, the helmet on the shelf over them. Faces +z; backs onto a wall.
  local k = maker()
  k.size = V(0.90, 1.95, 0.60)
  local tan, band = V(0.62, 0.52, 0.30), V(0.85, 0.85, 0.30)
  for _, x in ipairs({ -0.43, 0.43 }) do k.box("metal", V(x, 0.975, 0.30), V(0.04, 1.95, 0.58), 0.0, STEEL, 1) end   -- the sides
  k.box("metal", V(0, 1.65, 0.30), V(0.86, 0.03, 0.56), 0.0, STEEL, 1)                 -- the shelf
  k.box("metal", V(0, 0.06, 0.30), V(0.86, 0.03, 0.56), 0.0, STEEL, 1)                 -- the floor
  k.box("metal", V(0, 0.975, 0.03), V(0.86, 1.95, 0.02), 0.0, STEEL * 0.8, 1)          -- the back
  k.box("fabric", V(0, 1.20, 0.25), V(0.56, 0.75, 0.24), 0.05, tan)                    -- the coat
  k.box("fabric", V(0, 1.02, 0.38), V(0.58, 0.06, 0.02), 0.0, band, 1)                 -- its reflective band
  k.box("fabric", V(0, 0.42, 0.30), V(0.48, 0.30, 0.30), 0.06, tan * 0.9)              -- the trousers over the boots
  for _, x in ipairs({ -0.12, 0.12 }) do k.box("hard", V(x, 0.20, 0.32), V(0.16, 0.30, 0.28), 0.04, BLACK) end   -- the boots
  k.turned("hard", V(0, 1.67, 0.30), { { 0.0, 0.0 }, { 0.20, 0.0 }, { 0.17, 0.03 }, { 0.13, 0.12 }, { 0.07, 0.18 }, { 0.0, 0.19 } },
           V(0.80, 0.12, 0.08), 1, 12)                                                 -- the helmet
  return k.done()
end

M.machine = function(v)
  -- A MACHINE TOOL (a factory floor's: a CNC mill or a lathe, by the style dice): a painted steel base, the working
  -- enclosure over it with its window, the control pendant on its arm, a warning-striped plinth, a stack light. Faces
  -- +z: its operator stands there.
  local k = maker()
  local lathe = (styleOf(v) & 1) ~= 0
  local paint = lathe and V(0.30, 0.42, 0.36) or V(0.78, 0.78, 0.74)
  local W = lathe and 2.6 or 2.0
  k.size = V(W, 2.15, 1.6)
  k.box("hard", V(0, 0.04, 0.80), V(W, 0.08, 1.56), 0.0, V(0.85, 0.70, 0.10), 1)                -- the plinth
  k.box("metal", V(0, 0.50, 0.75), V(W - 0.1, 0.84, 1.4), 0.01, paint * 0.8, 1)                -- the base
  k.box("metal", V(0, 1.40, 0.70), V(W - 0.2, 0.96, 1.2), 0.03, paint, 1)                      -- the enclosure
  k.box("hard", V(lathe and -0.2 or 0, 1.42, 1.31), V(W * 0.5, 0.55, 0.02), 0.0, V(0.20, 0.25, 0.28), 1)   -- its window
  k.box("hard", V(W * 0.5 - 0.15, 1.35, 1.36), V(0.30, 0.45, 0.10), 0.01, BLACK * 2.0, 1)       -- the pendant
  k.rod("metal", V(W * 0.5 - 0.4, 1.95, 0.4), V(W * 0.5 - 0.4, 2.12, 0.4), 0.02, STEEL, 6)      -- the stack light's post
  k.box("light", V(W * 0.5 - 0.4, 2.08, 0.4), V(0.07, 0.06, 0.07), 0.0, V(0.2, 0.9, 0.3), 1)
  return k.done()
end

M.workbench = function(v)
  -- A WORKBENCH: a thick timber top on a steel frame, a vice at one end, a pegboard of tools behind it on its own
  -- uprights, a drawer unit under one end. Backs onto a wall (+z the worker's side).
  local k, wood = maker(), woodOf(v)
  k.size = V(2.0, 1.85, 0.80)
  k.box("wood", V(0, 0.88, 0.45), V(2.0, 0.06, 0.70), 0.006, wood, 1)
  for _, x in ipairs({ -0.94, 0.94 }) do
    for _, z in ipairs({ 0.14, 0.76 }) do k.box("metal", V(x, 0.43, z), V(0.05, 0.86, 0.05), 0.0, STEEL * 0.6, 1) end
  end
  k.box("metal", V(0.6, 0.45, 0.45), V(0.6, 0.70, 0.62), 0.006, V(0.70, 0.15, 0.12), 1)   -- drawers
  k.box("metal", V(-0.75, 0.98, 0.70), V(0.16, 0.14, 0.16), 0.01, V(0.20, 0.30, 0.55), 1) -- the vice
  k.box("hard", V(0, 1.42, 0.04), V(1.9, 0.86, 0.02), 0.0, V(0.75, 0.68, 0.52), 1)        -- the pegboard
  for i = 0, 5 do                                                                          -- its tools
    k.box("metal", V(-0.75 + i * 0.3, 1.40 + (i % 2) * 0.12, 0.07), V(0.04, 0.26, 0.03), 0.0, STEEL * (0.8 + 0.05 * i), 1)
  end
  return k.done()
end

M.pallet_stack = function(v)
  -- A PALLET of goods on the floor: the timber pallet, cartons stacked on it, shrink-wrapped.
  local k, wood = maker(), woodOf(v)
  k.size = V(1.2, 1.45, 1.0)
  k.box("wood", V(0, 0.07, 0.5), V(1.2, 0.14, 1.0), 0.0, wood * 1.1, 1)
  local layers = 2 + (styleOf(v) & 1)
  for l = 0, layers - 1 do
    k.box("hard", V(0, 0.14 + 0.21 + l * 0.42, 0.5), V(1.14, 0.40, 0.94), 0.01, V(0.62, 0.48, 0.32) * (0.92 + 0.05 * l), 1)
  end
  k.box("hard", V(0, 0.14 + layers * 0.21, 0.5), V(1.17, layers * 0.42, 0.97), 0.0, V(0.85, 0.87, 0.90), 1)   -- the wrap
  return k.done()
end

M.forklift = function(v)
  -- A FORKLIFT: the counterweighted body, the overhead guard on its posts, the seat and wheel, the mast at the front
  -- (+z) with its forks down on the floor.
  local k = maker()
  k.size = V(1.2, 2.2, 2.9)
  local yellow = V(0.90, 0.68, 0.10)
  k.box("hard", V(0, 0.55, 1.0), V(1.1, 0.80, 1.8), 0.03, yellow, 1)                     -- body
  k.box("hard", V(0, 0.75, 0.15), V(1.1, 0.90, 0.30), 0.03, BLACK * 2.5, 1)             -- counterweight
  for _, x in ipairs({ -0.48, 0.48 }) do
    for _, z in ipairs({ 0.35, 1.70 }) do k.box("metal", V(x, 1.55, z), V(0.06, 1.25, 0.06), 0.0, BLACK, 1) end   -- guard posts
  end
  k.box("metal", V(0, 2.17, 1.02), V(1.04, 0.05, 1.42), 0.0, BLACK, 1)                 -- the guard
  k.box("fabric", V(0, 1.08, 0.75), V(0.50, 0.12, 0.50), 0.03, BLACK * 2.0)            -- the seat
  k.box("metal", V(0, 1.62, 2.00), V(0.80, 1.16, 0.10), 0.0, STEEL * 0.6, 1)           -- the mast
  k.box("metal", V(0, 1.10, 2.00), V(0.90, 2.0, 0.08), 0.0, STEEL * 0.5, 1)
  for _, x in ipairs({ -0.30, 0.30 }) do k.box("metal", V(x, 0.04, 2.45), V(0.12, 0.05, 0.90), 0.0, STEEL * 0.7, 1) end   -- forks
  for _, x in ipairs({ -0.50, 0.50 }) do
    for _, z in ipairs({ 0.45, 1.65 }) do
      k.turned("hard", V(x, 0, z), { { 0.0, 0.0 }, { 0.12, 0.0 }, { 0.12, 0.30 }, { 0.0, 0.30 } }, BLACK, 1, 10)   -- wheels
    end
  end
  return k.done()
end

M.water_cooler = function(v)
  -- A WATER COOLER (Glenn, 2026-10-16: "a water cooler and so on"): a white cabinet, the drip tray and its two taps
  -- (cold blue, hot red) at cup height, the big blue bottle upside down on top. Faces +z.
  local k = maker()
  k.size = V(0.34, 1.34, 0.36)
  k.box("hard", V(0, 0.50, 0.18), V(0.32, 1.0, 0.32), 0.012, WHITE, 1)
  k.box("hard", V(0, 0.80, 0.335), V(0.22, 0.16, 0.012), 0.0, BLACK * 2.5, 1)          -- the recess
  k.box("hard", V(0, 0.73, 0.33), V(0.22, 0.02, 0.06), 0.0, STEEL * 0.7, 1)            -- its drip tray
  k.box("hard", V(-0.05, 0.86, 0.33), V(0.04, 0.04, 0.05), 0.004, V(0.15, 0.35, 0.80), 1)   -- cold
  k.box("hard", V(0.05, 0.86, 0.33), V(0.04, 0.04, 0.05), 0.004, V(0.80, 0.15, 0.12), 1)    -- hot
  k.box("hard", V(0, 1.005, 0.18), V(0.26, 0.01, 0.26), 0.0, STEEL * 0.8, 1)           -- the collar
  k.turned("hard", V(0, 1.0, 0.18), { { 0.0, 0.0 }, { 0.03, 0.0 }, { 0.03, 0.04 }, { 0.13, 0.09 }, { 0.135, 0.29 }, { 0.12, 0.33 },
                                      { 0.0, 0.335 } }, V(0.45, 0.68, 0.88), 1, 16)
  return k.done()
end

M.copier = function(v)
  -- A COPIER (the copy room's): a printer-copier on its paper-drawer base -- two drawers, the body with its output tray
  -- and a sheet in it, the scanner's lid on top, the control panel tilted toward the user (+z).
  local k = maker()
  k.size = V(0.62, 1.16, 0.68)
  local shell = V(0.82, 0.82, 0.80)
  k.box("hard", V(0, 0.31, 0.33), V(0.60, 0.58, 0.62), 0.008, shell * 0.92, 1)
  k.front("hard", -0.28, 0.28, 0.06, 0.30, 0.64, shell, BLACK * 3.0)
  k.front("hard", -0.28, 0.28, 0.32, 0.56, 0.64, shell, BLACK * 3.0)
  k.box("hard", V(0, 0.81, 0.32), V(0.60, 0.42, 0.62), 0.01, shell, 1)                  -- the body
  k.box("hard", V(-0.04, 0.86, 0.50), V(0.46, 0.08, 0.30), 0.0, BLACK * 2.0, 1)        -- the output slot
  k.box("ceramic", V(-0.04, 0.905, 0.47), V(0.21, 0.004, 0.28), 0.0, PORCELAIN, 1)     -- a sheet in it
  k.box("hard", V(0, 1.06, 0.31), V(0.58, 0.06, 0.56), 0.008, shell * 1.05, 1)          -- the scanner lid
  k.tilted("hard", V(0.17, 1.06, 0.61), V(0.24, 0.03, 0.12), 0.004, 0.45, BLACK * 2.0) -- the panel
  return k.done()
end

M.phone_booth = function(v)
  -- A PHONE BOOTH (an office's acoustic pod): felt-lined walls and a roof with its light, a glass door on the front
  -- (+z), a shelf to lean a laptop on and a stool inside. Stood in a row against a wall.
  local k, wood, fabric = maker(), woodOf(v), fabricOf(v)
  k.size = V(1.0, 2.25, 1.0)
  local felt = fabric * 0.8 + grey(0.08)
  k.box("hard", V(0, 0.03, 0.5), V(1.0, 0.06, 1.0), 0.0, STEEL * 0.6, 1)               -- the plinth
  k.box("fabric", V(0, 1.12, 0.03), V(1.0, 2.12, 0.06), 0.01, felt)                      -- the back
  for _, x in ipairs({ -0.47, 0.47 }) do k.box("fabric", V(x, 1.12, 0.5), V(0.06, 2.12, 1.0), 0.01, felt) end   -- sides
  k.box("hard", V(0, 2.21, 0.5), V(1.0, 0.08, 1.0), 0.01, WHITE, 1)                     -- the roof
  k.box("light", V(0, 2.165, 0.5), V(0.5, 0.01, 0.5), 0.0, V(1.0, 0.95, 0.85), 1)       -- its light
  k.box("wood", V(0, 1.02, 0.22), V(0.84, 0.03, 0.36), 0.004, wood, 1)                  -- the shelf
  k.turned("hard", V(0, 0, 0.62), { { 0.0, 0.0 }, { 0.18, 0.0 }, { 0.18, 0.02 }, { 0.03, 0.04 }, { 0.03, 0.62 }, { 0.17, 0.64 },
                                    { 0.17, 0.68 }, { 0.0, 0.69 } }, BLACK * 3.0, 1, 12)  -- the stool
  k.box("metal", V(0, 1.12, 0.975), V(0.88, 2.06, 0.012), 0.0, V(0.55, 0.62, 0.66), 1) -- the glass door
  k.rod("metal", V(0.36, 0.9, 0.995), V(0.36, 1.4, 0.995), 0.012, CHROME, 8)
  return k.done()
end

M.fitting_room = function(v)
  -- A FITTING ROOM (a department store's): a cubicle of painted panels clear of the floor, a long mirror on the back, a
  -- bench and a hook, the curtain on its rail across the front (+z), drawn back to one side. Stood in a row against a
  -- wall; the panels collide, not the floor inside: you step in.
  local k, wood, fabric = maker(), woodOf(v), fabricOf(v)
  k.size = V(1.0, 2.2, 1.3)
  k.colliders = { { V(0, 1.1, 0.02), V(1.0, 2.0, 0.04) }, { V(-0.48, 1.1, 0.65), V(0.04, 2.0, 1.3) },
                  { V(0.48, 1.1, 0.65), V(0.04, 2.0, 1.3) } }
  local panel = WHITE * 0.95
  k.box("hard", V(0, 1.12, 0.02), V(1.0, 1.96, 0.04), 0.005, panel, 1)                    -- the back
  for _, x in ipairs({ -0.48, 0.48 }) do k.box("hard", V(x, 1.12, 0.65), V(0.04, 1.96, 1.3), 0.005, panel, 1) end   -- sides
  for _, x in ipairs({ -0.48, 0.48 }) do k.box("metal", V(x, 0.07, 1.27), V(0.04, 0.14, 0.04), 0.0, STEEL, 1) end  -- feet
  k.box("metal", V(0, 1.15, 0.045), V(0.46, 1.5, 0.01), 0.0, CHROME, 1)                  -- the mirror
  k.box("wood", V(0, 0.45, 0.20), V(0.86, 0.04, 0.32), 0.004, wood, 1)                    -- the bench
  for _, x in ipairs({ -0.4, 0.4 }) do k.box("wood", V(x, 0.22, 0.20), V(0.04, 0.44, 0.30), 0.0, wood * 0.85, 1) end
  k.rod("metal", V(0.40, 1.70, 0.04), V(0.40, 1.70, 0.12), 0.01, CHROME, 6)              -- the hook
  k.rod("metal", V(-0.46, 2.08, 1.26), V(0.46, 2.08, 1.26), 0.012, CHROME, 8)            -- the rail
  k.box("fabric", V(0.32, 1.13, 1.26), V(0.30, 1.88, 0.07), 0.02, fabric)                 -- the curtain
  return k.done()
end

-- ---- THE BEACH (Glenn, 2026-11-02: "a towel under them and an umbrella. Or lounge chairs"): the colourway is the
-- variant's low three bits, brighter than the upholstery's.
M.sun_lounger = function(v)
  -- A SUN LOUNGER: a white aluminium frame on four legs, a padded bed, its head end (z = 0) raised a little; lain on
  -- feet first toward +z (the sea).
  local k = maker()
  local pad = BEACH[(v & 7) + 1]
  k.size = V(0.70, 0.52, 1.95)
  k.collider_h = 0.36
  for _, x in ipairs({ -0.31, 0.31 }) do
    k.rod("metal", V(x, 0.30, 0.05), V(x, 0.30, 1.90), 0.016, WHITE, 8)                  -- side rail
    for _, z in ipairs({ 0.18, 1.78 }) do k.rod("metal", V(x, 0.0, z), V(x, 0.30, z), 0.016, WHITE, 8) end   -- legs
  end
  k.box("fabric", V(0, 0.35, 1.22), V(0.62, 0.07, 1.30), 0.025, pad)                         -- the bed
  k.tilted("fabric", V(0, 0.41, 0.32), V(0.62, 0.07, 0.58), 0.025, 0.25, pad * 0.94)         -- the head rest
  return k.done()
end

M.beach_umbrella = function(v)
  -- A BEACH UMBRELLA: a white pole, a shallow canopy 2.2 m across with a valance round its rim, a finial.
  local k = maker()
  local cloth = BEACH[(v & 7) + 1]
  k.size = V(2.2, 2.32, 2.2)
  k.colliders = { { V(0, 1.15, 1.1), V(0.06, 2.3, 0.06) } }   -- only the pole: you walk under it
  k.rod("metal", V(0, 0.0, 1.1), V(0, 2.26, 1.1), 0.022, WHITE, 8)
  -- the canopy, its top (bottom to top) and the underside, darker, the other way
  k.turned("fabric", V(0, 1.94, 1.1), { { 1.1, 0.0 }, { 0.6, 0.22 }, { 0.0, 0.33 } }, cloth, 1, 16)
  k.turned("fabric", V(0, 1.935, 1.1), { { 0.0, 0.32 }, { 0.6, 0.21 }, { 1.09, 0.0 } }, cloth * 0.55, 1, 16)
  k.turned("fabric", V(0, 1.86, 1.1), { { 1.1, 0.0 }, { 1.1, 0.08 } }, WHITE, 1, 16)            -- the valance
  k.turned("fabric", V(0, 1.86, 1.1), { { 1.095, 0.08 }, { 1.095, 0.0 } }, WHITE * 0.6, 1, 16)
  k.turned("hard", V(0, 2.26, 1.1), { { 0.0, 0.0 }, { 0.03, 0.01 }, { 0.03, 0.04 }, { 0.0, 0.06 } }, WHITE, 1, 8)
  return k.done()
end

M.beach_towel = function(v)
  -- A BEACH TOWEL laid out on the sand: its colour, a white band across each end; lain on head at z = 0.
  local k = maker()
  local cloth = BEACH[(v & 7) + 1]
  k.size = V(0.85, 0.03, 1.75)
  k.solid = false
  k.box("fabric", V(0, 0.012, 0.875), V(0.85, 0.012, 1.75), 0.0, cloth, 1)
  for _, z in ipairs({ 0.18, 1.57 }) do k.box("fabric", V(0, 0.020, z), V(0.85, 0.006, 0.10), 0.0, WHITE, 1) end
  return k.done()
end
