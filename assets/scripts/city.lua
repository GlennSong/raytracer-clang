-- city.lua — author buildings with the split/shape grammar from Lua.
--
-- The grammar (building.grow) is an L1 engine object exposed to Lua, a sibling of
-- the L-system that grows flora (ADR-0028 §3, ADR-0038). Lua sets the params and
-- composes; the rewrite/emit loop stays hot in C++. This mirrors flora.lua: small
-- factory helpers over a C++ generator, returning meshes the loader can place.
--
-- The full road→block→parcel→building city is generated C++-side (shape:"city"
-- in a level); this file is the per-building authoring surface — a place to grow
-- one building, vary it, or kit-bash districts from script.

local city = {}

-- A mid-rise mixed-use block: taller glassy ground floor with a walkable
-- entrance, windowed residential floors, parapet roof (the ADR-0038 §7 hero).
function city.midrise(seed)
  return building.grow{
    floors = 4 + (seed or 0) % 4,
    width = 18, depth = 14,
    ground_retail = true,
    walkable_ground = true,
    bay_width = 3.6,
    seed = seed or 1,
  }
end

-- A downtown tower: many floors, narrow bays, stepped setbacks.
function city.tower(seed)
  return building.grow{
    floors = 20 + (seed or 0) % 20,
    width = 26, depth = 22,
    ground_retail = true,
    walkable_ground = true,
    bay_width = 4.2,
    setback_floors = 8,
    setback_every = 3.0,
    seed = seed or 1,
  }
end

-- A low residential house: a couple of storeys, no retail, smaller windows.
function city.house(seed)
  return building.grow{
    floors = 1 + (seed or 0) % 2,
    width = 12, depth = 10,
    ground_retail = false,
    walkable_ground = true,
    bay_width = 3.0,
    seed = seed or 1,
  }
end

-- Default recipe entry point: a representative mid-rise (so `runProcgenMesh` on
-- this file yields a building for preview / spawning).
return city.midrise(7)
