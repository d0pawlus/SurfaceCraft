// SurfaceCraft viewport: lit triangle meshes and unlit lines into an offscreen target.

struct Globals {
    view_proj: mat4x4<f32>,
    eye: vec4<f32>,
    light: vec4<f32>,
};

@group(0) @binding(0) var<uniform> globals: Globals;

struct VsIn {
    @location(0) pos: vec3<f32>,
    @location(1) normal: vec3<f32>,
    @location(2) color: vec4<f32>,
};

struct VsOut {
    @builtin(position) clip: vec4<f32>,
    @location(0) world: vec3<f32>,
    @location(1) normal: vec3<f32>,
    @location(2) color: vec4<f32>,
};

@vertex
fn vs_main(v: VsIn) -> VsOut {
    var o: VsOut;
    o.clip = globals.view_proj * vec4<f32>(v.pos, 1.0);
    o.world = v.pos;
    o.normal = v.normal;
    o.color = v.color;
    return o;
}

@fragment
fn fs_main(i: VsOut) -> @location(0) vec4<f32> {
    let n_len = length(i.normal);
    if (n_len < 0.001) {
        return i.color;
    }
    var n = i.normal / n_len;
    let to_eye = normalize(globals.eye.xyz - i.world);
    // two-sided lighting: scans are often open surfaces
    if (dot(n, to_eye) < 0.0) {
        n = -n;
    }
    let l = normalize(globals.light.xyz);
    let diffuse = max(dot(n, l), 0.0);
    let head = max(dot(n, to_eye), 0.0);
    let h = normalize(l + to_eye);
    let spec = pow(max(dot(n, h), 0.0), 48.0) * 0.25;
    let shade = 0.18 + 0.5 * diffuse + 0.35 * head;
    return vec4<f32>(i.color.rgb * shade + vec3<f32>(spec), i.color.a);
}
