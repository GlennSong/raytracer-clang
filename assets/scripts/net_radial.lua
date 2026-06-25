-- net_radial.lua — a procedural RADIAL street network (city.layout pattern="radial"):
-- concentric ring roads + radial avenues around a central roundabout. Flat, rendered
-- as a connected road surface with markings. Baseline for the city-generation work.
local lay = city.layout{ pattern = "radial", extent = 210, ring_spacing = 52, spokes = 12,
                         jitter = 0.05, seed = 5 }
local m = model.new()
m:add_solid(city.road_mesh(lay, { lift = 0.28, color = { 0.07, 0.07, 0.08 },
              sidewalk = 2.4, curb = 0.16, markings = true, mark_width = 0.18, crosswalks = true }))
return m
