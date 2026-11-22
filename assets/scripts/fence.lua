-- A FENCE ALONG A PATH (#191): posts and panels laid along a line of points on the ground, straight from point to
-- point or curved through them, with gaps left where gates go.
--
--   points  = { { x, z }, ... }   the line, in metres from the fence's own origin (two points or more)
--   curved  = true                run a smooth curve through the points (mesh.path.spline); else straight runs with corners
--   closed  = true                join the last point back to the first (a fenced yard)
--   style   = "picket" | "rail" | "board" | "iron" | "chain_link" | "wall" | "hedge"
--   height  = metres              (0 or absent: the style's own)
--   bay     = metres              the most a panel spans between two posts (absent: the style's own)
--   paint   = a palette paint     (picket, board, iron, chain_link's posts); wood = a palette wood (rail, or picket/board left bare)
--   gates   = { { at = metres along the line, width = metres }, ... }   gaps; each gets a marker `gate_1`, `gate_2` ...
--             at its middle on the ground, its +x along the fence, for a gate prefab to be stood on
--
-- Posts stand on y = 0. A straight fence puts a post on every corner; a curved one spaces them evenly. Each bay has
-- one collision shape. On flat ground only, for now.
local kit = require "model_kit"
local V = kit.V
local o = args or {}
local STYLES = {
  picket     = { height = 1.0, bay = 2.0, thick = 0.09 },
  rail       = { height = 1.2, bay = 2.4, thick = 0.12 },
  board      = { height = 1.8, bay = 2.4, thick = 0.1 },
  iron       = { height = 1.4, bay = 2.2, thick = 0.07 },
  chain_link = { height = 2.0, bay = 3.0, thick = 0.06 },
  wall       = { height = 0.8, bay = 3.0, thick = 0.3 },
  hedge      = { height = 1.2, bay = 3.0, thick = 0.7 },
}
local style = o.style or "picket"
local S = STYLES[style] or error("fence: no style named '" .. tostring(style) .. "' (picket, rail, board, iron, chain_link, wall, hedge)")
local H = (o.height and o.height > 0) and o.height or S.height
local BAY = (o.bay and o.bay > 0) and o.bay or S.bay
local pts = o.points or { { -3, 0 }, { 3, 0 } }
if #pts < 2 then error("fence: `points` needs two points or more") end

local WOOD = kit.wood(o.wood or "oak")
local PAINT = o.paint and kit.paint[o.paint] or nil
local IRON, STEEL = kit.metal.iron, kit.metal.steel
local BRICK, STONE, LEAF = V(0.55, 0.3, 0.24), V(0.72, 0.7, 0.66), V(0.16, 0.34, 0.14)

-- THE LINE, as points close enough together to walk along: the corners themselves, or a curve through them
local line = {}
if o.curved and #pts >= 3 then
  local three = {}
  for i, p in ipairs(pts) do three[i] = { p[1], 0, p[2] } end
  for i, p in ipairs(mesh.path.spline(three, { count = math.max(24, #pts * 12), closed = o.closed and true or false })) do line[i] = { p[1], p[3] } end
else
  for i, p in ipairs(pts) do line[i] = { p[1], p[2] } end
  if o.closed then line[#line + 1] = { pts[1][1], pts[1][2] } end
end
local along = { 0 }                                -- how far along the line each point is
for i = 2, #line do
  along[i] = along[i - 1] + math.sqrt((line[i][1] - line[i - 1][1]) ^ 2 + (line[i][2] - line[i - 1][2]) ^ 2)
end
local TOTAL = along[#line]
local function at(s)                               -- the point s metres along, and which way the line runs there
  s = math.max(0, math.min(TOTAL, s))
  for i = 2, #line do
    if s <= along[i] or i == #line then
      local span = along[i] - along[i - 1]
      local t = span > 0 and (s - along[i - 1]) / span or 0
      local p, q = line[i - 1], line[i]
      return { p[1] + (q[1] - p[1]) * t, p[2] + (q[2] - p[2]) * t }, { (q[1] - p[1]) / math.max(span, 1e-9), (q[2] - p[2]) / math.max(span, 1e-9) }
    end
  end
end

-- THE RUNS: the stretches between gates. A straight fence also ends a run at every corner, so a post stands there.
local cuts = { 0, TOTAL }
local gates = {}
for _, g in ipairs(o.gates or {}) do
  local w = g.width or 1.2
  local s0, s1 = math.max(0, g.at - w / 2), math.min(TOTAL, g.at + w / 2)
  gates[#gates + 1] = { s0, s1 }
  cuts[#cuts + 1], cuts[#cuts + 2] = s0, s1
end
if not (o.curved and #pts >= 3) then for i = 2, #line - 1 do cuts[#cuts + 1] = along[i] end end
table.sort(cuts)
local function in_gate(s) for _, g in ipairs(gates) do if s > g[1] + 1e-6 and s < g[2] - 1e-6 then return true end end return false end

local m = model.new()
local far = m:level(70)
local near, low = kit.maker(), kit.maker()

-- ONE BAY'S PANEL in its own space: from x = 0 to x = L, standing on y = 0, its middle on z = 0
local function panel(k, L)
  if style == "picket" then
    local col = PAINT or WOOD
    for _, y in ipairs{ 0.25, H - 0.25 } do k.box("wood", V(L / 2, y, -0.03), V(L, 0.07, 0.03), 0.004, col, 1) end
    local n = math.max(1, math.floor(L / 0.13))
    for i = 1, n do
      local x = (i - 0.5) * L / n
      k.box("wood", V(x, (H - 0.06) / 2 + 0.03, 0), V(0.075, H - 0.09, 0.02), 0.0, col, 1)
      k.mesh("wood", mesh.translate(mesh.scale(mesh.rotate_y(mesh.lathe{ profile = { { 0, 0 }, { 0.053, 0 }, { 0, 0.06 } }, segments = 4 }, math.pi / 4), { 1, 1, 0.27 }), { x, H - 0.015, 0 }), col)
    end
  elseif style == "rail" then
    for _, y in ipairs{ H * 0.3, H * 0.6, H * 0.9 } do k.box("wood", V(L / 2, y, 0), V(L, 0.11, 0.035), 0.006, PAINT or WOOD, 1) end
  elseif style == "board" then
    local col = PAINT or WOOD
    for _, y in ipairs{ 0.3, H - 0.3 } do k.box("wood", V(L / 2, y, -0.035), V(L, 0.09, 0.04), 0.004, col, 1) end
    local n = math.max(1, math.floor(L / 0.15))
    for i = 1, n do k.box("wood", V((i - 0.5) * L / n, H / 2 + 0.02, 0), V(L / n - 0.004, H - 0.04, 0.02), 0.0, col * (0.94 + 0.06 * ((i * 7) % 3)), 1) end
  elseif style == "iron" then
    local col = PAINT or IRON
    for _, y in ipairs{ 0.15, H - 0.18 } do k.box("metal", V(L / 2, y, 0), V(L, 0.035, 0.02), 0.002, col, 1) end
    local n = math.max(1, math.floor(L / 0.13))
    for i = 1, n do
      local x = (i - 0.5) * L / n
      k.rod("metal", V(x, 0.06, 0), V(x, H - 0.07, 0), 0.008, col, 4)
      k.mesh("metal", mesh.translate(mesh.lathe{ profile = { { 0, 0 }, { 0.02, 0.02 }, { 0, 0.09 } }, segments = 4 }, { x, H - 0.08, 0 }), col)
    end
  elseif style == "chain_link" then
    k.rod("metal", V(0, H - 0.03, 0), V(L, H - 0.03, 0), 0.02, STEEL, 6)
    k.rod("metal", V(0, 0.06, 0), V(L, 0.06, 0), 0.012, STEEL, 6)
    k.box("mesh", V(L / 2, H / 2, 0), V(L, H - 0.08, 0.004), 0.0, V(0.62, 0.65, 0.66), 1)
  elseif style == "wall" then
    k.box("masonry", V(L / 2, (H - 0.08) / 2, 0), V(L, H - 0.08, 0.24), 0.0, PAINT or BRICK, 1)
    k.box("stone", V(L / 2, H - 0.04, 0), V(L, 0.08, 0.32), 0.01, STONE, 1)
  else   -- hedge
    k.box("leaf", V(L / 2, H / 2, 0), V(L, H, S.thick), 0.12, LEAF, 2)
  end
end
local function post(k, x, z)
  if style == "picket" or style == "board" then
    k.box("wood", V(x, (H + 0.08) / 2, z), V(0.09, H + 0.08, 0.09), 0.006, PAINT or WOOD, 1)
  elseif style == "rail" then
    k.box("wood", V(x, (H + 0.1) / 2, z), V(0.13, H + 0.1, 0.13), 0.01, PAINT or WOOD, 1)
  elseif style == "iron" then
    k.box("metal", V(x, (H + 0.05) / 2, z), V(0.06, H + 0.05, 0.06), 0.004, PAINT or IRON, 1)
    k.mesh("metal", mesh.translate(kit.ball(0.045, 8), { x, H + 0.09, z }), PAINT or IRON)
  elseif style == "chain_link" then
    k.rod("metal", V(x, 0, z), V(x, H, z), 0.03, PAINT or STEEL, 8)
  elseif style == "wall" then
    k.box("masonry", V(x, H / 2 + 0.05, z), V(0.36, H + 0.1, 0.36), 0.0, PAINT or BRICK, 1)
    k.box("stone", V(x, H + 0.14, z), V(0.44, 0.08, 0.44), 0.01, STONE, 1)
  end
end

local panels = {}                                  -- a bay's panel by its length (to the centimetre): made once
local function panel_lists(L)
  local key = math.floor(L * 100 + 0.5)
  if not panels[key] then
    local k = kit.maker()
    panel(k, L)
    local merged = {}
    for part, list in pairs(k.lists) do merged[part] = mesh.merge(list) end
    panels[key] = merged
  end
  return panels[key]
end

local posted, bays = {}, 0
local function stand_post(p)
  local key = math.floor(p[1] * 50 + 0.5) .. "," .. math.floor(p[2] * 50 + 0.5)
  if posted[key] then return end
  posted[key] = true
  post(near, p[1], p[2])
end
for c = 1, #cuts - 1 do
  local s0, s1 = cuts[c], cuts[c + 1]
  if s1 - s0 > 0.05 and not in_gate((s0 + s1) / 2) then
    local n = math.max(1, math.ceil((s1 - s0) / BAY - 1e-6))
    for i = 1, n do
      local a0, a1 = s0 + (s1 - s0) * (i - 1) / n, s0 + (s1 - s0) * i / n
      local p, q = at(a0), at(a1)
      local dx, dz = q[1] - p[1], q[2] - p[2]
      local L = math.sqrt(dx * dx + dz * dz)
      local turn = math.atan(-dz, dx)              -- about +y, taking the panel's own +x onto the bay
      stand_post(p); stand_post(q)
      for part, made in pairs(panel_lists(L)) do
        near.mesh(part, mesh.translate(mesh.rotate_y(made, turn), { p[1], 0, p[2] }))
      end
      local mid = { (p[1] + q[1]) / 2, H / 2, (p[2] + q[2]) / 2 }
      -- far off: a solid style is one slab; an open one (railings, rails, chain link) is only its rails, so the
      -- fence does not turn into a wall at a distance
      local function slab(part, y0, y1, thick, col)
        low.mesh(part, mesh.translate(mesh.rotate_y(mesh.translate(mesh.box{ L, y1 - y0, thick }, { L / 2, 0, 0 }), turn), { p[1], (y0 + y1) / 2, p[2] }), col)
      end
      if style == "hedge" then slab("leaf", 0, H, S.thick, LEAF)
      elseif style == "wall" then slab("masonry", 0, H, 0.24, PAINT or BRICK)
      elseif style == "board" then slab("wood", 0, H, 0.04, PAINT or WOOD)
      elseif style == "picket" then slab("wood", 0.2, H - 0.2, 0.03, PAINT or WOOD)
      elseif style == "rail" then for _, y in ipairs{ H * 0.3, H * 0.6, H * 0.9 } do slab("wood", y - 0.055, y + 0.055, 0.035, PAINT or WOOD) end
      elseif style == "iron" then for _, y in ipairs{ 0.15, H - 0.18 } do slab("metal", y - 0.03, y + 0.03, 0.03, PAINT or IRON) end
      else slab("metal", H - 0.06, H, 0.04, STEEL); slab("mesh", 0.04, H - 0.06, 0.004, V(0.62, 0.65, 0.66)) end
      bays = bays + 1
      m:shape("bay_" .. bays, { box = { L, H, math.max(0.1, S.thick) }, pos = mid, turn = { 0, math.deg(turn), 0 } })
    end
  end
end
kit.put(m, m, near)
kit.put(m, m, low, { level = far })
for i, g in ipairs(gates) do
  local p, d = at((g[1] + g[2]) / 2)
  m:marker("gate_" .. i, { pos = { p[1], 0, p[2] }, turn = { 0, math.deg(math.atan(-d[2], d[1])), 0 }, role = "stand" })
end
return m
