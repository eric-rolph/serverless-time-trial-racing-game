// Flat-shaded lambert + ambient, one directional light.

struct Globals {
    view_proj: mat4x4<f32>,
    // xyz = direction the light travels (normalized), w unused
    light_dir: vec4<f32>,
};

struct ObjectU {
    model: mat4x4<f32>,
    color: vec4<f32>,
};

@group(0) @binding(0) var<uniform> globals: Globals;
@group(1) @binding(0) var<uniform> obj: ObjectU;

struct VsOut {
    @builtin(position) clip: vec4<f32>,
    @location(0) world_normal: vec3<f32>,
};

@vertex
fn vs_main(@location(0) pos: vec3<f32>, @location(1) normal: vec3<f32>) -> VsOut {
    var out: VsOut;
    let world = obj.model * vec4<f32>(pos, 1.0);
    out.clip = globals.view_proj * world;
    // model matrices are rigid (rotation+translation), so w=0 transform is fine
    out.world_normal = (obj.model * vec4<f32>(normal, 0.0)).xyz;
    return out;
}

@fragment
fn fs_main(in: VsOut) -> @location(0) vec4<f32> {
    let n = normalize(in.world_normal);
    let l = normalize(-globals.light_dir.xyz);
    // two-sided lambert (abs) so ribbon/terrain winding never renders black
    let lambert = abs(dot(n, l));
    let ambient = 0.28;
    let lit = obj.color.rgb * (ambient + (1.0 - ambient) * lambert);
    return vec4<f32>(lit, obj.color.a);
}
