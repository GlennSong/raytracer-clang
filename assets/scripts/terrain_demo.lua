-- terrain_demo.lua — terrain composed from primitives (ADR-0043), not a preset:
-- rolling fbm hills with ridged mountains lifted in, domain-warped so the
-- ridgelines read organic instead of banded. Baked to a mesh and returned as a
-- model — the same pipeline that renders buildings and props.

local hills = terrain.fbm{ seed = 7,  freq = 0.012, amp = 14, octaves = 5 }
local mtns  = terrain.ridged{ seed = 11, freq = 0.005, amp = 46, octaves = 6 }

-- Hills everywhere, mountains added at half weight, then domain-warped.
local land  = terrain.warp(hills:add(mtns:scale(0.5)),
                           terrain.fbm{ seed = 3, freq = 0.02, amp = 1 }, 32)

local mesh = terrain.mesh(land, { size = 400, resolution = 220,
                                  color = {0.34, 0.42, 0.26} })

local m = model.new()
m:add(mesh)
return m
