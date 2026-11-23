// APEX Native wgpu Cook-Torrance PBR + Physical Atmospheric Sky Shader.
// Features:
//   • Full-screen physical sky dome (Rayleigh gradient, Mie solar glow, sun disc,
//     procedural drifting clouds) rendered cleanly at depth 1.0;
//   • Cook-Torrance GGX PBR (sun key light + hemisphere sky/ground IBL + analytic
//     specular sky reflection so metallic car paint gleams natively);
//   • Procedural world-space asphalt aggregate grain & turf mottling;
//   • Distance atmospheric horizon fog + ACES Filmic tone mapping.

struct Globals {
    view_proj: mat4x4<f32>,
    inv_view_proj: mat4x4<f32>,
    cam_pos: vec4<f32>,      // xyz = camera world pos, w = elapsed time (s)
    light_dir: vec4<f32>,    // xyz = direction light travels (normalized), w = sun intensity
    sky_top: vec4<f32>,      // rgb = zenith color, w = fog near
    sky_horizon: vec4<f32>,  // rgb = horizon / fog color, w = fog far
};

struct ObjectU {
    model: mat4x4<f32>,
    color: vec4<f32>,
    // x = roughness, y = metalness, z = surface kind (0 std, 1 asphalt, 2 turf, 3 unlit/emissive), w = emissive
    material: vec4<f32>,
};

@group(0) @binding(0) var<uniform> globals: Globals;
@group(1) @binding(0) var<uniform> obj: ObjectU;

const PI: f32 = 3.14159265;

fn aces_filmic(x: vec3<f32>) -> vec3<f32> {
    let a = 2.51;
    let b = 0.03;
    let c = 2.43;
    let d = 0.59;
    let e = 0.14;
    return clamp((x * (a * x + b)) / (x * (c * x + d) + e), vec3<f32>(0.0), vec3<f32>(1.0));
}

fn hash21(p_in: vec2<f32>) -> f32 {
    var p = fract(p_in * vec2<f32>(234.34, 435.345));
    p += dot(p, p + 34.23);
    return fract(p.x * p.y);
}

fn noise2(p: vec2<f32>) -> f32 {
    let i = floor(p);
    let f_raw = fract(p);
    let f = f_raw * f_raw * (3.0 - 2.0 * f_raw);
    let a = hash21(i);
    let b = hash21(i + vec2<f32>(1.0, 0.0));
    let c = hash21(i + vec2<f32>(0.0, 1.0));
    let d = hash21(i + vec2<f32>(1.0, 1.0));
    return mix(mix(a, b, f.x), mix(c, d, f.x), f.y);
}

fn fbm(p_in: vec2<f32>) -> f32 {
    var p = p_in;
    var v = 0.0;
    var a = 0.5;
    for (var i = 0; i < 4; i++) {
        v += a * noise2(p);
        p = vec2<f32>(p.x * 0.8 - p.y * 0.6, p.x * 0.6 + p.y * 0.8) * 2.03 + vec2<f32>(11.3, 5.7);
        a *= 0.5;
    }
    return v;
}

fn sample_sky(dir: vec3<f32>, include_clouds: bool) -> vec3<f32> {
    let d = normalize(dir);
    let sun_dir = normalize(-globals.light_dir.xyz);
    let elev = clamp(d.y, 0.0, 1.0);
    let ground_col = vec3<f32>(0.11, 0.13, 0.12);
    var col = select(
        mix(globals.sky_horizon.rgb, ground_col, pow(clamp(-d.y, 0.0, 1.0), 0.45)),
        mix(globals.sky_horizon.rgb, globals.sky_top.rgb, pow(elev, 0.58)),
        d.y >= 0.0,
    );

    let cs = max(dot(d, sun_dir), 0.0);
    let sun_tint = vec3<f32>(1.0, 0.82, 0.58);
    col += sun_tint * (pow(cs, 6.0) * 0.45 + pow(cs, 48.0) * 0.85);
    if (include_clouds) {
        // Crisp solar disc
        col += vec3<f32>(3.0, 2.5, 1.9) * smoothstep(0.9985, 0.9996, cs);
        // Drifting planar horizon clouds
        if (d.y > 0.02) {
            let t = globals.cam_pos.w;
            let uv = d.xz / (d.y + 0.15) * 0.72 + vec2<f32>(t * 0.008, t * 0.003);
            let dens = fbm(uv);
            let mask = smoothstep(0.54, 0.78, dens) * smoothstep(0.02, 0.22, d.y);
            let cloud_lit = mix(vec3<f32>(0.42, 0.36, 0.44), vec3<f32>(1.0, 0.90, 0.78), 0.5 + 0.5 * pow(cs, 3.0));
            col = mix(col, cloud_lit, mask * 0.72);
        }
    }
    return col;
}

// ---------------------------------------------------------------------------
// Full-screen atmospheric sky pass (3-vertex triangle, no vertex buffer)
// ---------------------------------------------------------------------------

struct SkyOut {
    @builtin(position) clip: vec4<f32>,
    @location(0) ndc: vec2<f32>,
};

@vertex
fn vs_sky(@builtin(vertex_index) vid: u32) -> SkyOut {
    var pos = array<vec2<f32>, 3>(
        vec2<f32>(-1.0, -1.0),
        vec2<f32>(3.0, -1.0),
        vec2<f32>(-1.0, 3.0),
    );
    var out: SkyOut;
    let p = pos[vid];
    out.clip = vec4<f32>(p, 0.99999, 1.0);
    out.ndc = p;
    return out;
}

@fragment
fn fs_sky(in: SkyOut) -> @location(0) vec4<f32> {
    let p_far = globals.inv_view_proj * vec4<f32>(in.ndc, 1.0, 1.0);
    let world_dir = normalize(p_far.xyz / p_far.w - globals.cam_pos.xyz);
    let col = aces_filmic(sample_sky(world_dir, true));
    return vec4<f32>(col, 1.0);
}

// ---------------------------------------------------------------------------
// Cook-Torrance PBR world geometry pass
// ---------------------------------------------------------------------------

struct VsOut {
    @builtin(position) clip: vec4<f32>,
    @location(0) world_pos: vec3<f32>,
    @location(1) world_normal: vec3<f32>,
};

@vertex
fn vs_main(@location(0) pos: vec3<f32>, @location(1) normal: vec3<f32>) -> VsOut {
    var out: VsOut;
    let world = obj.model * vec4<f32>(pos, 1.0);
    out.clip = globals.view_proj * world;
    out.world_pos = world.xyz;
    out.world_normal = (obj.model * vec4<f32>(normal, 0.0)).xyz;
    return out;
}

@fragment
fn fs_main(in: VsOut) -> @location(0) vec4<f32> {
    let view_vec = globals.cam_pos.xyz - in.world_pos;
    let dist = length(view_vec);
    let v = view_vec / max(dist, 1e-4);

    // Ensure two-sided winding faces the camera so ribbon/terrain never render black
    var n = normalize(in.world_normal);
    if (dot(n, v) < 0.0) {
        n = -n;
    }

    var albedo = obj.color.rgb;
    var roughness = clamp(obj.material.x, 0.08, 1.0);
    let metalness = clamp(obj.material.y, 0.0, 1.0);
    let kind = obj.material.z;
    let emissive = obj.material.w;

    // Procedural world-space micro-detail (asphalt aggregate / grass turf)
    if (kind > 0.5 && kind < 1.5) {
        let g1 = noise2(in.world_pos.xz * 2.8);
        let g2 = noise2(in.world_pos.xz * 14.0);
        let grain = (g1 * 0.65 + g2 * 0.35) - 0.5;
        albedo *= 1.0 + grain * 0.24;
        roughness = clamp(roughness + grain * 0.14, 0.35, 0.98);
    } else if (kind >= 1.5 && kind < 2.5) {
        let turf = fbm(in.world_pos.xz * 0.18) - 0.5;
        let fine = noise2(in.world_pos.xz * 1.9) - 0.5;
        albedo *= 1.0 + turf * 0.32 + fine * 0.12;
    }

    let l = normalize(-globals.light_dir.xyz);
    let h = normalize(v + l);
    let ndotl = max(dot(n, l), 0.0);
    let ndotv = max(dot(n, v), 0.001);
    let ndoth = max(dot(n, h), 0.0);
    let vdoth = max(dot(v, h), 0.0);

    // Cook-Torrance GGX direct sun term
    let a = roughness * roughness;
    let a2 = a * a;
    let denom = ndoth * ndoth * (a2 - 1.0) + 1.0;
    let d_ggx = a2 / max(PI * denom * denom, 1e-5);
    let k = (roughness + 1.0) * (roughness + 1.0) / 8.0;
    let g_v = ndotv / (ndotv * (1.0 - k) + k);
    let g_l = ndotl / (ndotl * (1.0 - k) + k);
    let g_smith = g_v * g_l;

    let f0 = mix(vec3<f32>(0.04), albedo, metalness);
    let fresnel = f0 + (1.0 - f0) * pow(1.0 - vdoth, 5.0);

    let spec = (d_ggx * g_smith * fresnel) / max(4.0 * ndotv * ndotl, 0.001);
    let kd = (1.0 - fresnel) * (1.0 - metalness);
    let sun_radiance = vec3<f32>(1.0, 0.84, 0.66) * globals.light_dir.w;
    let direct = (kd * albedo / PI + spec) * sun_radiance * ndotl;

    // Hemisphere diffuse + analytic sky specular IBL reflection
    let hemi = mix(vec3<f32>(0.16, 0.17, 0.15), globals.sky_horizon.rgb * 0.65, n.y * 0.5 + 0.5);
    let refl_dir = reflect(-v, n);
    let env_spec = sample_sky(refl_dir, false) * (1.0 - roughness * 0.78);
    let f_env = f0 + (max(vec3<f32>(1.0 - roughness), f0) - f0) * pow(1.0 - ndotv, 5.0);
    let ambient = kd * albedo * hemi * 0.95 + env_spec * f_env * 0.72;

    var lit = direct + ambient + albedo * emissive;

    // Aerial distance fog blending smoothly into horizon
    let fog_near = globals.sky_top.w;
    let fog_far = globals.sky_horizon.w;
    let fog_t = clamp((dist - fog_near) / max(fog_far - fog_near, 1.0), 0.0, 1.0);
    lit = mix(lit, globals.sky_horizon.rgb, fog_t * fog_t * 0.88);

    let mapped = aces_filmic(lit);
    return vec4<f32>(mapped, obj.color.a);
}
