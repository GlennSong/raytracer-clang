-- PREFAB TOWN: the place the test level `prefab_town` stands its prefabs in (#191; docs/model-record-design.md,
-- "The test level"). A street, a pavement each side, and rooms along both: each room a floor and four walls with a
-- doorway in the wall that faces the street, no ceiling, so it can be walked into and seen from above. A park and
-- a paved square at one end. It makes NO furniture: every thing in the level is a prefab placed by the level file,
-- which tools/prefab_town.py writes, and the rooms come from the same table (the level's `opts`).
--
-- These rooms are stand-ins. The city's real rooms are grown by the building grammar and furnished by the floor
-- planner; that is the next thing to bring here, when the planner places prefabs.
--
--   opts.rooms    { { name, x, z, w, d, yaw (0: the doorway faces +z; 180: -z), door (the gap's width), floor }, ... }
--   opts.street   { x0, x1, half (the carriageway's half width), walk (a pavement's width) }
--   opts.grounds  { { x, z, w, d, kind = "grass" | "paving" }, ... }
local o = args or {}
local m = model.new()
local WALL_H, WALL_T = 3.0, 0.15

local MATS = {
  asphalt = material.new{ surface = "pavement", roughness = 0.95 },
  walk    = material.new{ surface = "concrete", roughness = 0.9 },
  wall    = material.new{ roughness = 0.85 },
  wood    = material.new{ surface = "wood", roughness = 0.55 },
  tile    = material.new{ surface = "concrete", roughness = 0.35 },
  carpet  = material.new{ surface = "carpet", roughness = 0.95 },
  grass   = material.new{ roughness = 1.0 },
  paving  = material.new{ surface = "concrete", roughness = 0.8 },
  line    = material.new{ roughness = 0.7 },
}
local TINT = {
  asphalt = { 0.16, 0.16, 0.17 }, walk = { 0.6, 0.6, 0.58 }, wall = { 0.9, 0.89, 0.85 }, wood = { 0.62, 0.45, 0.28 },
  tile = { 0.8, 0.82, 0.82 }, carpet = { 0.35, 0.4, 0.48 }, grass = { 0.3, 0.5, 0.24 }, paving = { 0.7, 0.67, 0.6 },
  line = { 0.9, 0.9, 0.85 },
}
local function slab(kind, x0, x1, z0, z1, y)
  m:add(mesh.paint(mesh.translate(mesh.box{ x1 - x0, 0.02, z1 - z0 }, { (x0 + x1) / 2, y - 0.01, (z0 + z1) / 2 }), TINT[kind]), MATS[kind])
end

local s = o.street or { x0 = -30, x1 = 30, half = 4, walk = 3 }
slab("asphalt", s.x0, s.x1, -s.half, s.half, 0.02)
slab("walk", s.x0, s.x1, -s.half - s.walk, -s.half, 0.04)
slab("walk", s.x0, s.x1, s.half, s.half + s.walk, 0.04)
for x = s.x0 + 2, s.x1 - 4, 6 do slab("line", x, x + 3, -0.07, 0.07, 0.025) end
for _, g in ipairs(o.grounds or {}) do slab(g.kind, g.x - g.w / 2, g.x + g.w / 2, g.z - g.d / 2, g.z + g.d / 2, 0.03) end

local walls = 0
for _, r in ipairs(o.rooms or {}) do
  local side = r.yaw == 180 and -1 or 1                     -- the room's own +z (toward its doorway) in the world
  slab(r.floor or "wood", r.x - r.w / 2, r.x + r.w / 2, r.z - r.d / 2, r.z + r.d / 2, 0.05)
  -- a wall from (ax, az) to (bx, bz) in the room's own space
  local function wall(ax, az, bx, bz)
    local wx, wz = math.abs(bx - ax) + WALL_T, math.abs(bz - az) + WALL_T
    local at = { r.x + side * (ax + bx) / 2, WALL_H / 2, r.z + side * (az + bz) / 2 }
    m:add(mesh.paint(mesh.translate(mesh.box{ wx, WALL_H, wz }, at), TINT.wall), MATS.wall)
    walls = walls + 1
    m:shape(string.format("wall_%d", walls), { box = { wx, WALL_H, wz }, pos = at })
  end
  local hw, hd, gap = r.w / 2, r.d / 2, (r.door or 1.02) / 2
  wall(-hw, -hd, hw, -hd)                                   -- the back
  wall(-hw, -hd, -hw, hd)
  wall(hw, -hd, hw, hd)
  wall(-hw, hd, -gap, hd)                                   -- the front, either side of the doorway
  wall(gap, hd, hw, hd)
end
return m
