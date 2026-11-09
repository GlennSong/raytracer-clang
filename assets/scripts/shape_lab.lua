-- shape_lab.lua -- THE CURVE AND SURFACE WORDS laid out for a look (#191; Glenn, 2026-11-06: "We need to be able to
-- go further than a box for shapes ... curves, rounded objects, and more"). Rows, front to back: profiles extruded
-- with soft edges; shapes turned on the lathe (revolve); profiles swept along curves; lofts through sections;
-- superquadrics from box to sphere to star; and furniture made of nothing but these words. Hot-reloads.
local m = model.new()

local MATS = {
  paint = material.new{ roughness = 0.45 },
  metal = material.new{ roughness = 0.25, metallic = 0.9 },
  wood  = material.new{ surface = "woodgrain", roughness = 0.55 },
  floor = material.new{ roughness = 0.8 },
}
local function put(mesh_, x, z, mat) m:add(mesh.translate(mesh_, { x, 0, z }), MATS[mat or "paint"]) end

-- a smooth OPEN profile (r, y) through points: a lathe's outline from a spline
local function outline(points, count)
  local p3 = {}
  for i, q in ipairs(points) do p3[i] = { q[1], q[2], 0 } end
  local path = mesh.path.spline(p3, { count = count or 48 })
  local prof = { closed = false }
  for i, q in ipairs(path) do prof[i] = { q[1], q[2] } end
  return prof
end

local C = {
  red = { 0.78, 0.16, 0.12 }, orange = { 0.90, 0.48, 0.12 }, yellow = { 0.92, 0.78, 0.20 }, green = { 0.25, 0.60, 0.32 },
  teal = { 0.12, 0.55, 0.60 }, blue = { 0.20, 0.36, 0.75 }, violet = { 0.50, 0.30, 0.70 }, white = { 0.90, 0.90, 0.88 },
  steel = { 0.75, 0.76, 0.78 }, brass = { 0.82, 0.66, 0.30 },
}

-- ROW 1, z = 0: EXTRUDE -- rectangle, rounded rectangle, circle, squircle (superellipse n = 4), star, hexagon; each
-- 0.4 m tall with its top and bottom edges rounded
local row1 = {
  { mesh.profile.rect(1.0, 0.7), 0.0, C.red },
  { mesh.profile.rounded_rect(1.0, 0.7, 0.2, 6), 0.06, C.orange },
  { mesh.profile.circle(0.45, 48), 0.06, C.yellow },
  { mesh.profile.superellipse(0.5, 0.4, 4, 64), 0.06, C.green },
  { mesh.profile.star(5, 0.55, 0.25), 0.03, C.teal },
  { mesh.profile.polygon(6, 0.5), 0.05, C.blue },
}
for i, r in ipairs(row1) do put(mesh.extrude(r[1], 0.4, { bevel = r[2], bevel_segs = 4, color = r[3] }), (i - 1) * 1.6, 0) end

-- ROW 2, z = 2.5: REVOLVE -- a vase, a bowl, a goblet, a dome cut to three quarters, a torus (a circle turned off the
-- axis), a finial
put(mesh.revolve(outline{ { 0, 0 }, { 0.22, 0.0 }, { 0.32, 0.25 }, { 0.18, 0.62 }, { 0.14, 0.78 }, { 0.2, 0.9 } }, { segs = 48, color = C.teal }), 0, 2.5)
put(mesh.revolve(mesh.profile.open{ { 0, 0 }, { 0.18, 0 }, { 0.40, 0.18 }, { 0.48, 0.26 }, { 0.46, 0.28 }, { 0.36, 0.20 }, { 0.16, 0.06 }, { 0, 0.06 } },
                 { segs = 48, color = C.white }), 1.6, 2.5)
put(mesh.revolve(outline({ { 0, 0 }, { 0.2, 0.0 }, { 0.05, 0.08 }, { 0.03, 0.4 }, { 0.22, 0.55 }, { 0.26, 0.8 } }, 64),
                 { segs = 48, color = C.brass }), 3.2, 2.5, "metal")
do
  -- a dome 3 cm thick (a closed profile: the outer arc up, the inner one back down), cut to three quarters
  local shell = { closed = true }
  for k = 0, 16 do local a = (math.pi / 2) * k / 16; shell[#shell + 1] = { 0.5 * math.cos(a), 0.5 * math.sin(a) } end
  for k = 16, 0, -1 do local a = (math.pi / 2) * k / 16; shell[#shell + 1] = { 0.47 * math.cos(a), 0.47 * math.sin(a) } end
  put(mesh.revolve(shell, { segs = 48, from = 0, to = math.pi * 1.5, color = C.violet }), 4.8, 2.5)
end
do
  local ring = mesh.profile.circle(0.14, 24)
  for _, q in ipairs(ring) do q[1] = q[1] + 0.36; q[2] = q[2] + 0.16 end
  put(mesh.revolve(ring, { segs = 64, color = C.red }), 6.4, 2.5)
end
put(mesh.revolve(outline{ { 0, 0 }, { 0.12, 0.0 }, { 0.14, 0.1 }, { 0.05, 0.25 }, { 0.16, 0.45 }, { 0.1, 0.62 }, { 0.02, 0.8 }, { 0, 0.85 } },
                 { segs = 32, color = C.steel }), 8.0, 2.5, "metal")

-- ROW 3, z = 5: SWEEP -- a spring (a helix), a square-section ring, a horn tapering and twisting, a handrail on a
-- spline, a cable sagging on a Bezier, a moulding (an open profile along a corner)
put(mesh.sweep(mesh.profile.circle(0.04, 12), mesh.path.helix({ 0, 0.05, 0 }, 0.3, 0.12, 6, 40), { color = C.steel }), 0, 5, "metal")
put(mesh.sweep(mesh.profile.rect(0.12, 0.12), mesh.path.circle({ 0, 0.52, 0 }, 0.45, { 0, 0, 1 }, 64), { color = C.orange }), 1.6, 5)
put(mesh.sweep(mesh.profile.star(6, 0.18, 0.09),
               mesh.path.spline({ { 0, 0, 0 }, { 0.1, 0.4, 0 }, { -0.15, 0.8, 0.1 }, { 0.1, 1.1, 0 } }, { count = 48 }),
               { scale = { 1, 0.15 }, twist = math.pi, color = C.violet }), 3.2, 5)
put(mesh.sweep(mesh.profile.circle(0.03, 12),
               mesh.path.spline({ { -0.6, 0.9, -0.2 }, { -0.2, 1.0, 0.1 }, { 0.2, 0.85, -0.1 }, { 0.6, 1.0, 0.2 } }, { count = 64 }),
               { color = C.brass }), 4.8, 5, "metal")
put(mesh.sweep(mesh.profile.circle(0.015, 8), mesh.path.bezier({ -0.6, 1.0, 0 }, { -0.2, 0.2, 0 }, { 0.2, 0.2, 0 }, { 0.6, 1.0, 0 }, 48),
               { color = { 0.1, 0.1, 0.1 } }), 6.4, 5)
do
  -- an ogee moulding's edge (open), swept round two sides of a square
  local ogee = mesh.profile.open{ { 0, 0 }, { 0.06, 0 }, { 0.06, 0.02 }, { 0.04, 0.05 }, { 0.02, 0.06 }, { 0.02, 0.09 }, { 0, 0.09 } }
  put(mesh.sweep(ogee, mesh.path.spline({ { -0.5, 0, 0.5 }, { -0.5, 0, -0.5 }, { 0.5, 0, -0.5 } }, { count = 48 }),
                 { up = { 0, 1, 0 }, color = C.white }), 8.0, 5, "wood")
end

-- ROW 4, z = 7.5: LOFT -- a boat's hull through ellipse halves, a bottle through circles, a car-ish body through
-- rounded rectangles
do
  local secs = {}
  for i = 0, 8 do
    local t = i / 8
    local w = 0.05 + 0.4 * math.sin(math.pi * t) ^ 0.6
    local d = 0.05 + 0.25 * math.sin(math.pi * t) ^ 0.5
    secs[#secs + 1] = mesh.section(mesh.profile.ellipse(w, d, 24), { origin = { 0, 0.35, -0.8 + 1.6 * t }, right = { 1, 0, 0 }, up = { 0, 1, 0 } })
  end
  put(mesh.loft(secs, { color = C.blue }), 0, 7.5)
end
do
  local radii = { { 0.18, 0 }, { 0.2, 0.05 }, { 0.2, 0.45 }, { 0.12, 0.62 }, { 0.06, 0.7 }, { 0.06, 0.82 }, { 0.07, 0.84 } }
  local secs = {}
  for _, rr in ipairs(radii) do
    secs[#secs + 1] = mesh.section(mesh.profile.circle(rr[1], 32), { origin = { 0, rr[2], 0 }, right = { 1, 0, 0 }, up = { 0, 0, 1 } })
  end
  put(mesh.loft(secs, { color = C.green }), 1.6, 7.5)
end
do
  local secs = {}
  local rows = { { 0.5, 0.25, 0.10, -1.0 }, { 0.8, 0.40, 0.18, -0.6 }, { 0.85, 0.65, 0.25, 0.0 }, { 0.82, 0.55, 0.22, 0.5 }, { 0.7, 0.30, 0.12, 1.0 } }
  for _, r in ipairs(rows) do
    secs[#secs + 1] = mesh.section(mesh.profile.rounded_rect(r[1], r[2], r[3], 6), { origin = { 0, r[2] * 0.5 + 0.1, r[4] }, right = { 1, 0, 0 }, up = { 0, 1, 0 } })
  end
  put(mesh.loft(secs, { color = C.red }), 3.6, 7.5)
end

-- ROW 5, z = 10: SUPERQUADRICS -- exponents from a soft box (0.2) through the sphere (1) to a star-like diamond (2.5)
local E = { { 0.2, 0.2 }, { 0.5, 0.5 }, { 1, 1 }, { 0.3, 1 }, { 1, 0.3 }, { 2.5, 2.5 } }
for i, e in ipairs(E) do
  put(mesh.translate(mesh.superquadric({ 0.4, 0.3, 0.4 }, e[1], e[2], { color = row1[i][3] }), { 0, 0.3, 0 }), (i - 1) * 1.6, 10)
end

-- ROW 6, z = 13: FURNITURE FROM THE WORDS -- a chair, a round table, a lamp: no box anywhere
do
  -- the chair: a soft seat (extrude), four turned legs (revolve), a back of two swept posts and a bent rail
  local x0 = 0
  put(mesh.translate(mesh.extrude(mesh.profile.rounded_rect(0.46, 0.44, 0.06, 6), 0.05, { bevel = 0.015, bevel_segs = 3, color = C.white }),
                     { 0, 0.43, 0 }), x0, 13, "wood")
  local leg = mesh.revolve(outline{ { 0, 0 }, { 0.016, 0 }, { 0.02, 0.15 }, { 0.014, 0.3 }, { 0.018, 0.43 }, { 0, 0.43 } }, { segs = 16, color = C.white })
  for _, p in ipairs{ { -0.19, -0.18 }, { 0.19, -0.18 }, { -0.19, 0.18 }, { 0.19, 0.18 } } do
    put(mesh.translate(leg, { p[1], 0, p[2] }), x0, 13, "wood")
  end
  for _, sx in ipairs{ -0.19, 0.19 } do
    put(mesh.sweep(mesh.profile.circle(0.014, 10), mesh.path.spline({ { sx, 0.47, 0.18 }, { sx, 0.7, 0.21 }, { sx, 0.9, 0.24 } }, { count = 16 }),
                   { color = C.white }), x0, 13, "wood")
  end
  put(mesh.sweep(mesh.profile.rounded_rect(0.06, 0.02, 0.008, 3),
                 mesh.path.spline({ { -0.21, 0.86, 0.24 }, { 0, 0.88, 0.29 }, { 0.21, 0.86, 0.24 } }, { count = 24 }), { color = C.white }),
      x0, 13, "wood")
end
do
  -- the table: a round top with a soft edge, a turned pedestal, a cross foot of swept curves
  local x0 = 2.2
  put(mesh.translate(mesh.extrude(mesh.profile.circle(0.55, 64), 0.04, { bevel = 0.012, bevel_segs = 3, color = C.white }), { 0, 0.72, 0 }), x0, 13, "wood")
  put(mesh.revolve(outline{ { 0, 0.08 }, { 0.05, 0.08 }, { 0.035, 0.3 }, { 0.06, 0.5 }, { 0.04, 0.72 }, { 0, 0.72 } }, { segs = 32, color = C.white }), x0, 13, "wood")
  for k = 0, 3 do
    local a = k * math.pi / 2
    local c, s = math.cos(a), math.sin(a)
    put(mesh.sweep(mesh.profile.rounded_rect(0.05, 0.04, 0.012, 3),
                   mesh.path.bezier({ 0, 0.1, 0 }, { 0.15 * c, 0.1, 0.15 * s }, { 0.3 * c, 0.02, 0.3 * s }, { 0.36 * c, 0.02, 0.36 * s }, 24),
                   { color = C.white }), x0, 13, "wood")
  end
end
do
  -- the lamp: a weighted base (revolve), a spiral stem (helix sweep), a shade (a part-open revolve)
  local x0 = 4.4
  put(mesh.revolve(outline{ { 0, 0 }, { 0.14, 0 }, { 0.15, 0.02 }, { 0.1, 0.05 }, { 0.02, 0.06 }, { 0, 0.06 } }, { segs = 48, color = C.steel }), x0, 13, "metal")
  put(mesh.sweep(mesh.profile.circle(0.008, 8), mesh.path.helix({ 0, 0.06, 0 }, 0.02, 0.1, 8, 24), { color = C.brass }), x0, 13, "metal")
  put(mesh.revolve(mesh.profile.open{ { 0.07, 1.15 }, { 0.2, 0.9 }, { 0.205, 0.89 }, { 0.075, 1.15 } }, { segs = 48, color = C.yellow }), x0, 13)
end

-- ROW 7, z = 16: MODIFIERS -- a rod bent into a hook, a star column tapered and twisted, posts round a circle (array
-- along a path), a spiral stair (an array turning as it climbs), a bracket and its mirror, a blob warped by a function
put(mesh.bend(mesh.sweep(mesh.profile.circle(0.03, 12), mesh.path.line({ 0, 0, 0 }, { 0, 1.0, 0 }, 48), { color = C.steel }), math.pi, 0.6),
    0, 16, "metal")
put(mesh.twist(mesh.taper(mesh.extrude(mesh.profile.star(6, 0.22, 0.15), 1.2, { bevel = 0.01, bevel_segs = 2, segs = 48, color = C.white }), 0.55, 1.2),
               1.6), 1.6, 16)
put(mesh.array_along(mesh.extrude(mesh.profile.rounded_rect(0.06, 0.06, 0.015, 2), 0.5, { color = C.brass }),
                     mesh.path.circle({ 0, 0, 0 }, 0.55, { 0, 1, 0 }, 64), 14, { align = false }), 3.2, 16, "metal")
put(mesh.array(mesh.translate(mesh.extrude(mesh.profile.rounded_rect(0.5, 0.18, 0.04, 3), 0.035, { bevel = 0.01, bevel_segs = 2, color = C.orange }),
                              { 0.3, 0, 0 }), 14, { offset = { 0, 0.09, 0 }, rotate_y = 0.42 }), 4.8, 16)
do
  local bracket = mesh.sweep(mesh.profile.rect(0.05, 0.03),
                             mesh.path.bezier({ 0.05, 0.0, 0 }, { 0.5, 0.0, 0 }, { 0.5, 0.6, 0 }, { 0.15, 0.8, 0 }, 32), { color = C.violet })
  put(bracket, 6.4, 16)
  put(mesh.mirror(bracket, { 1, 0, 0 }, 0), 6.4, 16)
end
put(mesh.warp(mesh.superquadric({ 0.35, 0.35, 0.35 }, 1, 1, { stacks = 32, slices = 64, color = C.teal }),
              function(p) local w = 1 + 0.35 * math.sin(p[2] * 9); return { p[1] * w, p[2] + 0.4, p[3] * w } end), 8.0, 16)

-- the floor
m:add(mesh.translate(mesh.extrude(mesh.profile.rect(12, 22), 0.05, { color = { 0.62, 0.62, 0.6 } }), { 4.8, -0.05, 8.5 }), MATS.floor)
return m
