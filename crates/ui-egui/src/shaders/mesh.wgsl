// SurfaceCraft viewport: lit triangle meshes and unlit lines into an offscreen target.

struct Globals {
    view_proj: mat4x4<f32>,
    eye: vec4<f32>,
    light: vec4<f32>,
    clip: vec4<f32>,
};

@group(0) @binding(0) var<uniform> globals: Globals;
struct Object {
    // preview motion of the object (identity when it is not being dragged)
    model: mat4x4<f32>,
    // colour of the back side of its triangles (alpha 0 = the same as the front)
    back: vec4<f32>,
};

@group(1) @binding(0) var<uniform> object: Object;

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
    let world = object.model * vec4<f32>(v.pos, 1.0);
    o.clip = globals.view_proj * world;
    o.world = world.xyz;
    o.normal = (object.model * vec4<f32>(v.normal, 0.0)).xyz;
    o.color = v.color;
    return o;
}

// Lit colour of a fragment whose surface colour is `color`.
fn lit(i: VsOut, color: vec4<f32>) -> vec4<f32> {
    // section plane: the far side of it is clipped away (all-zero plane = off)
    if (dot(globals.clip.xyz, i.world) - globals.clip.w > 0.0) {
        discard;
    }
    let n_len = length(i.normal);
    if (n_len < 0.001) {
        return color;
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
    return vec4<f32>(color.rgb * shade + vec3<f32>(spec), color.a);
}

// Meshes: the back side of a triangle (by its winding, which is what the normals are computed
// from) takes the object's back colour, so the two sides of a scan are told apart.
@fragment
fn fs_main(i: VsOut, @builtin(front_facing) is_front: bool) -> @location(0) vec4<f32> {
    if (!is_front && object.back.a > 0.0) {
        return lit(i, vec4<f32>(object.back.rgb, i.color.a));
    }
    return lit(i, i.color);
}

// Selection highlights and lines: one colour on both sides.
@fragment
fn fs_one_colour(i: VsOut) -> @location(0) vec4<f32> {
    return lit(i, i.color);
}
