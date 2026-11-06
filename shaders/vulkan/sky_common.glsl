// THE SKY'S HORIZON, BY DIRECTION (ADR-0339): shared by sky.frag (the sky you see) and mesh.frag (the light it casts), so
// the two agree. The day model's horizon colour (day_night_cycle.cpp) is one colour all round; at a low sun the real sky
// is warm only toward the sun and a cooler, dimmer blue-grey away from it. With one colour every wall facing away
// from the sun was filled as warm as the wall facing it -- buildings "illuminated all around" at sunset (Glenn,
// 2026-11-05). The tint grows as the sun nears the horizon and is gone by mid-morning.
//   dir    -- the direction looked or lit toward (unit)
//   horizon, zenith -- the day model's colours
//   sunDir -- toward the sun (unit)
vec3 skyHorizonToward(vec3 dir, vec3 horizon, vec3 zenith, vec3 sunDir) {
    const float dusk = (1.0 - smoothstep(0.02, 0.35, sunDir.y)) * smoothstep(-0.18, 0.0, sunDir.y);
    if (dusk <= 0.0) return horizon;
    const vec2 d = dir.xz, s = sunDir.xz;
    const float az = dot(d, s) / (length(d) * length(s) + 1e-5);        // -1 away .. 1 toward the sun
    const float toward = pow(0.5 + 0.5 * az, 1.6);
    const float lum = dot(horizon, vec3(0.2126, 0.7152, 0.0722));
    const vec3 away = mix(zenith, vec3(lum) * vec3(0.68, 0.73, 0.95), 0.5) * 0.8;   // the anti-twilight: cool, dimmer
    return mix(horizon, mix(away, horizon * 1.2, toward), dusk);
}
