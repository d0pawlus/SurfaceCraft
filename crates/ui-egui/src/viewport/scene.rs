//! What a frame draws: the objects, the camera and the overlay, and the colour state that tells when a
//! colour buffer must be rewritten.
use crate::camera::Camera;
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

/// A world-space model matrix as the shader takes it for vertices stored relative to `origin`:
/// column-major, with the shift folded in (`p' = M (q + o) - o`).
pub(super) fn shifted_model(rows: &[[f64; 4]; 4], o: &Point3) -> [[f32; 4]; 4] {
    let oo = [o.x, o.y, o.z];
    let mut out = [[0f32; 4]; 4];
    for r in 0..3 {
        for c in 0..3 {
            out[c][r] = rows[r][c] as f32;
        }
        let rot_o = rows[r][0] * oo[0] + rows[r][1] * oo[1] + rows[r][2] * oo[2];
        out[3][r] = (rot_o + rows[r][3] - oo[r]) as f32;
    }
    out[3][3] = 1.0;
    out
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

pub(super) fn colors_of(item: &DrawItem) -> Vec<[f32; 4]> {
    let n = item.mesh.positions.len();
    match &item.colors {
        Some(c) if c.len() == n => c.as_ref().clone(),
        _ => vec![item.color; n],
    }
}
