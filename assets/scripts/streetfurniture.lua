-- streetfurniture.lua — author street props with the street kit from Lua (ADR-0042).
--
-- Each helper wraps a C++ street-kit builder (streetfurniture.lamp /
-- streetfurniture.traffic_signal). Lua only sets the tunable params and composes;
-- the build loop stays hot in C++, exactly like city.lua / flora.lua. The same
-- mesh feeds the realtime viewer and the offline path tracer — the renderer is
-- the only divergent point (ADR-0042).

local sf = {}

-- A standard street lamp: a ~4.6 m pole with a warm head.
function sf.lamp(opts)
  opts = opts or {}
  return streetfurniture.lamp{
    height      = opts.height or 4.6,
    pole_radius = opts.pole_radius or 0.07,
    head_size   = opts.head_size or {0.5, 0.2, 0.32},
    head_color  = opts.head_color or {1.0, 0.88, 0.55},
  }
end

-- A taller boulevard lamp with a larger head, for wide avenues.
function sf.boulevard_lamp(opts)
  opts = opts or {}
  return streetfurniture.lamp{
    height      = opts.height or 6.2,
    pole_radius = 0.09,
    head_size   = {0.6, 0.24, 0.40},
    head_color  = opts.head_color or {1.0, 0.9, 0.6},
  }
end

-- A mast-arm traffic signal (built facing +Z; place with a yaw transform).
function sf.signal(opts)
  opts = opts or {}
  return streetfurniture.traffic_signal{
    pole_height = opts.pole_height or 5.6,
    arm_length  = opts.arm_length or 4.2,
    arm_height  = opts.arm_height or 5.3,
  }
end

-- Default recipe entry point: a representative lamp (so runProcgenMesh on this
-- file yields a mesh for preview / spawning).
return sf.lamp()
