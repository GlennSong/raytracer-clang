-- net_grid.lua — a procedural GRID street network (city.layout pattern="grid"),
-- flat, rendered as a connected road surface with multilane markings. Baseline for
-- the city-generation work (grids / radials / highway ramps).
local lay = city.layout{ pattern = "grid", extent = 200, cell_size = 66, jitter = 0.10, seed = 4 }
local m = model.new()
m:add_solid(city.road_mesh(lay, { lift = 0.28, color = { 0.07, 0.07, 0.08 },
              sidewalk = 2.4, curb = 0.16, markings = true, mark_width = 0.18, crosswalks = true }))
return m
