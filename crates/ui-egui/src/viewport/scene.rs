//! What a frame draws: the objects, the camera and the overlay, and the colour state that tells when a
//! colour buffer must be rewritten.
use crate::camera::Camera;
use std::borrow::Cow;
use std::sync::Arc;
use surfacecraft_geom::{Mesh, Point3};

/// What a frame adds to the objects: a clip plane and preview motions.
#[derive(Clone, Debug, Default)]
pub struct Overlay {
    /// Clip plane `(nx, ny, nz, d)` in world coordinates (fragments with `n · p > d` are
    /// discarded); all zero = no clipping.
    pub clip: [f64; 4],
    /// Preview motion of drawn items by `DrawItem::key`: row-major rigid world transforms (an
    /// object being dragged by the gizmo is drawn moved without touching its mesh).
    pub models: Vec<(usize, [[f64; 4]; 4])>,
}

/// A world-space model matrix as the shader takes it: for vertices stored relative to their mesh's
/// own origin `m`, giving positions relative to the frame's origin `o` (`p' = M (q + m) - o`),
/// column-major. The mesh keeps the origin it was uploaded with while the frame's follows the
/// camera, so a pan rewrites this matrix and not the vertices (until `origin_too_far`).
pub(super) fn shifted_model(rows: &[[f64; 4]; 4], m: &Point3, o: &Point3) -> [[f32; 4]; 4] {
    let (mm, oo) = ([m.x, m.y, m.z], [o.x, o.y, o.z]);
    let mut out = [[0f32; 4]; 4];
    for r in 0..3 {
        for c in 0..3 {
            out[c][r] = rows[r][c] as f32;
        }
        let rot_m = rows[r][0] * mm[0] + rows[r][1] * mm[1] + rows[r][2] * mm[2];
        out[3][r] = (rot_m + rows[r][3] - oo[r]) as f32;
    }
    out[3][3] = 1.0;
    out
}

/// Largest f32 rounding error of a drawn vertex, in pixels, before a mesh is uploaded again
/// around the frame's origin.
const MAX_ROUNDING_PX: f64 = 0.25;

/// Whether vertices stored in f32 relative to `mesh_origin` are too coarse for this frame: the
/// rounding step at the camera target (its distance from that origin times the f32 epsilon) is
/// a visible part of a pixel. A pan moves the target by pixels and this takes millions of them,
/// so panning keeps the upload; zooming far into a part far from the origin renews it once.
pub(super) fn origin_too_far(mesh_origin: &Point3, f: &Frame) -> bool {
    let step = (f.origin - mesh_origin).norm() * f64::from(f32::EPSILON);
    let pixel = f.camera.world_per_pixel(f.camera.distance, f64::from(f.size[1]));
    step > pixel * MAX_ROUNDING_PX
}

/// The identity as a world row-major matrix.
pub(super) fn identity_rows() -> [[f64; 4]; 4] {
    let mut m = [[0f64; 4]; 4];
    for (i, row) in m.iter_mut().enumerate() {
        row[i] = 1.0;
    }
    m
}

/// One object to draw this frame.
#[derive(Clone)]
pub struct DrawItem {
    /// Changes when the geometry changes (the GPU mesh is re-uploaded).
    pub key: usize,
    /// Geometry.
    pub mesh: Arc<Mesh>,
    /// Base colour.
    pub color: [f32; 4],
    /// Optional per-vertex colours.
    pub colors: Option<Arc<Vec<[f32; 4]>>>,
    /// Changes when the content of `colors` does (only the colour buffer is rewritten).
    pub colors_key: u64,
    /// Optional per-triangle highlight (selected triangles of `mesh`), drawn in the highlight colour.
    pub highlight: Option<Arc<Vec<u32>>>,
    /// Colour of the back side (where the normals point away from the eye); `None` draws both
    /// sides in the base colour. Per-vertex colours are drawn on both sides instead.
    pub back: Option<[f32; 4]>,
}

/// Everything the GPU needs for one frame.
#[derive(Clone)]
pub struct Frame {
    /// Viewport size in physical pixels.
    pub size: [u32; 2],
    /// Camera.
    pub camera: Camera,
    /// Precision origin (scene centre).
    pub origin: Point3,
    /// Objects.
    pub items: Vec<DrawItem>,
    /// Background colour.
    pub background: [f64; 3],
    /// Clip plane and preview motions.
    pub overlay: Overlay,
}
/// What the colour buffer of an item shows: its base colour and which colour array.
pub(super) fn color_state(c: &[f32; 4], has_colors: bool, key: u64) -> u64 {
    let mut h = 1469598103934665603u64;
    for v in c {
        h = (h ^ u64::from(v.to_bits())).wrapping_mul(1099511628211);
    }
    // fold each field in on its own: XOR-ing `has_colors` and `key` together made
    // (no colours, key 0) and (colours, key 1) - the first colour array ever shown - the same
    // state, so the first region/deviation colours were never uploaded
    for v in [u64::from(has_colors), key] {
        h = (h ^ v).wrapping_mul(1099511628211);
    }
    h
}

/// Whether the colour buffer must be rewritten: what it shows (`shown`) differs from what the
/// item wants now (new colours with the same geometry key included).
pub(super) fn colors_need_upload(shown: u64, item: &DrawItem) -> bool {
    shown != color_state(&item.color, item.colors.is_some(), item.colors_key)
}

/// The per-vertex colours of an item when they fit its mesh (others are not drawn).
fn vertex_colors(item: &DrawItem) -> Option<&Vec<[f32; 4]>> {
    item.colors.as_deref().filter(|c| c.len() == item.mesh.positions.len())
}

/// The colours to upload: the item's own per-vertex colours as they are (no copy of a
/// multi-megabyte array per upload), or its one colour on every vertex.
pub(super) fn colors_of(item: &DrawItem) -> Cow<'_, [[f32; 4]]> {
    match vertex_colors(item) {
        Some(c) => Cow::Borrowed(c.as_slice()),
        None => Cow::Owned(vec![item.color; item.mesh.positions.len()]),
    }
}

/// The back colour the shader gets: the item's, unless per-vertex colours are drawn (they are
/// data and stay the same on both sides). Alpha 0 turns the back colour off.
pub(super) fn back_of(item: &DrawItem) -> [f32; 4] {
    match (vertex_colors(item), item.back) {
        (None, Some(back)) => back,
        _ => [0.0; 4],
    }
}

/// The per-object uniform: where the object is drawn and the colour of its back side.
/// `mesh_origin` and `frame_origin` are those of `shifted_model`.
pub(super) fn object_uniform(
    rows: &[[f64; 4]; 4],
    mesh_origin: &Point3,
    frame_origin: &Point3,
    back: [f32; 4],
) -> ObjectUniform {
    ObjectUniform { model: shifted_model(rows, mesh_origin, frame_origin), back }
}

/// Per-object uniform of the mesh shader (`Object` in `mesh.wgsl`).
#[repr(C)]
#[derive(Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)]
pub(super) struct ObjectUniform {
    /// Column-major model matrix (the preview motion).
    model: [[f32; 4]; 4],
    /// Back-side colour; alpha 0 = none.
    back: [f32; 4],
}
