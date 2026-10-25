-- THE PARTICLE EFFECTS (engine/particles.h, ADR-0239). Any system sets one off by name
-- (requestParticles(world, "spark", at, along)); a ParticleEmitter, or a level's "emitters" entry, keeps one flowing.
-- Edit or add an effect here: no C++. An entry naming a built-in replaces the fields it gives.
--
--   gravity   m/s^2 down (negative rises: a bubble, steam)      drag      1/s of speed lost to the air
--   wobble    m/s of sideways wander                              life      s: a number, or {min, max}
--   shape     "puff" (soft), "ring" (a bubble), "glow" (a hot middle: sparks, streaks)
--   colour    {r, g, b} lit by the scene                          glow      {r, g, b} emitted (a spark's heat)
--   size      m across: a number, or {start, end} over the life   alpha     the most it shows (0..1)
--   fade_in   fraction of the life spent appearing                fade_out  ...and the last fraction going
--   stretch   s: drawn as a line this long along its speed (0: round)
--   a burst:  count, speed (m/s along the asked direction), up (m/s), spread (m/s, any way), jitter (m off the point)

particles = {
  -- kicked off the ground: tan, low, spreading and settling, gone in about a second
  dust = {
    gravity = 1.0, drag = 3.0, life = {0.7, 1.3},
    shape = "puff", colour = {0.62, 0.55, 0.45}, size = {0.18, 0.75}, alpha = 0.7, fade_in = 0.1, fade_out = 0.6,
    count = 8, speed = 1.0, up = 0.4, spread = 0.9, jitter = 0.15,
  },
  -- tyre smoke: big soft billows that hang, drift up a little and grow
  smoke = {
    gravity = -0.4, drag = 1.6, life = {1.4, 2.4},
    shape = "puff", colour = {0.82, 0.82, 0.84}, glow = {0.12, 0.12, 0.13}, size = {0.5, 2.6}, alpha = 0.75, fade_in = 0.12, fade_out = 0.65,
    count = 1, speed = 1.0, up = 0.5, spread = 0.7, jitter = 0.25,
  },
  -- under water: small rings rising (buoyant, held to ~0.7 m/s by the water), wobbling
  bubble = {
    gravity = -6.0, drag = 9.0, wobble = 0.15, life = {1.5, 3.0},
    shape = "ring", colour = {0.85, 0.95, 1.0}, glow = {0.05, 0.07, 0.08}, size = {0.035, 0.07}, alpha = 0.8,
    fade_in = 0.05, fade_out = 0.15,
    count = 1, speed = 0.3, spread = 0.15, jitter = 0.03,
  },
  -- a breath let out under water: a gulp of bubbles
  breath = {
    gravity = -6.0, drag = 9.0, wobble = 0.15, life = {1.5, 3.0},
    shape = "ring", colour = {0.85, 0.95, 1.0}, glow = {0.05, 0.07, 0.08}, size = {0.035, 0.07}, alpha = 0.8,
    fade_in = 0.05, fade_out = 0.15,
    count = 14, speed = 0.5, spread = 0.35, jitter = 0.06,
  },
  -- the air going past under a canopy / in a wingsuit: a faint line where it was, stretched against the body
  streak = {
    life = {0.5, 0.9},
    shape = "glow", colour = {1, 1, 1}, glow = {0.45, 0.45, 0.48}, size = 0.05, alpha = 0.38,
    fade_in = 0.3, fade_out = 0.4, stretch = 0.16,
    count = 1, speed = 0, spread = 0,
  },
  -- a hook biting, a truck grinding: hot, quick, falling, a line along its flight
  spark = {
    gravity = 9.81, drag = 1.0, life = {0.18, 0.45},
    shape = "glow", colour = {0, 0, 0}, glow = {6.0, 3.0, 0.9}, size = {0.03, 0.015}, alpha = 1.0,
    fade_in = 0.0, fade_out = 0.5, stretch = 0.025,
    count = 12, speed = 3.0, up = 1.5, spread = 4.0, jitter = 0.03,
  },
  -- into the water: white drops thrown up and falling back
  splash = {
    gravity = 9.81, drag = 0.6, life = {0.5, 0.9},
    shape = "glow", colour = {0.95, 0.97, 1.0}, glow = {0.35, 0.36, 0.38}, size = {0.09, 0.05}, alpha = 0.85,
    fade_in = 0.0, fade_out = 0.3, stretch = 0.02,
    count = 16, speed = 0, up = 2.5, spread = 1.4, jitter = 0.4,
  },
  -- a vent, a kettle, a manhole on a cold night: white wisps rising and spreading
  steam = {
    gravity = -1.2, drag = 1.2, life = {1.5, 3.0},
    shape = "puff", colour = {0.92, 0.93, 0.95}, glow = {0.18, 0.18, 0.19}, size = {0.25, 1.8}, alpha = 0.6, fade_in = 0.15, fade_out = 0.7,
    count = 1, speed = 0, up = 0.8, spread = 0.3, jitter = 0.15,
  },
}
