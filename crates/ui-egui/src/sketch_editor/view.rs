//! The sketch plane seen through the viewport's one camera: sketch coordinates to screen pixels
//! and back, and the camera alignment that looks straight onto the plane.
use crate::camera::Camera;
use surfacecraft_geom::{Mat4, Point3, Vec3};
use surfacecraft_sketch::{Sketch, SketchPlane};

/// Looks the camera onto `plane`: from its normal side, centred on `centre` (sketch
/// coordinates) and far enough to show `extent` (sketch units across).
pub fn align_camera(cam: &mut Camera, plane: &SketchPlane, centre: [f64; 2], extent: f64) {
    let n = plane.normal();
    cam.target = plane.to_3d(centre);
    cam.pitch = n.z.clamp(-1.0, 1.0).asin().clamp(-1.55, 1.55);
    if n.x.hypot(n.y) > 0.1 {
        cam.yaw = n.y.atan2(n.x);
    } else {
        // looking down (or up) the Z axis: put the plane's x axis to the right of the screen
        // (screen right is (-sin yaw, cos yaw, 0)); seen from below, mirrored
        let x = Vec3::from(plane.x);
        let sign = if n.z >= 0.0 { 1.0 } else { -1.0 };
        cam.yaw = (-x.x * sign).atan2(x.y * sign);
    }
    let half = (extent.max(1e-3) * 0.5) * 1.3;
    cam.distance = half / (cam.fov / 2.0).tan();
}

/// A bounding box of a sketch: centre and the larger extent (a default of 100 when empty).
#[must_use]
pub fn sketch_bounds(sk: &Sketch) -> ([f64; 2], f64) {
    if sk.points.is_empty() {
        return ([0.0, 0.0], 100.0);
    }
    let mut lo = [f64::INFINITY; 2];
    let mut hi = [f64::NEG_INFINITY; 2];
    for p in &sk.points {
        for k in 0..2 {
            lo[k] = lo[k].min(p[k]);
            hi[k] = hi[k].max(p[k]);
        }
    }
    let ext = (hi[0] - lo[0]).max(hi[1] - lo[1]);
    ([(lo[0] + hi[0]) / 2.0, (lo[1] + hi[1]) / 2.0], if ext < 1e-6 { 100.0 } else { ext })
}

/// Maps between sketch coordinates and viewport pixels (relative to the viewport's top-left).
pub struct PlaneView {
    cam: Camera,
    plane: SketchPlane,
    origin: Point3,
    m: Mat4,
    /// Viewport width in pixels (points).
    pub w: f32,
    /// Viewport height in pixels (points).
    pub h: f32,
}

impl PlaneView {
    /// The view of `plane` through `cam` in a `w × h` viewport.
    #[must_use]
    pub fn new(cam: &Camera, plane: &SketchPlane, w: f32, h: f32) -> Self {
        let origin = cam.target;
        let aspect = f64::from(w) / f64::from(h.max(1.0));
        let m = cam.projection(aspect) * cam.view(&origin);
        Self { cam: cam.clone(), plane: plane.clone(), origin, m, w, h }
    }

    /// Pixel of a 3-D point (`None` behind the camera).
    #[must_use]
    pub fn project(&self, p: &Point3) -> Option<[f32; 2]> {
        let v = self.m * (p - self.origin.coords).to_homogeneous();
        if v.w <= 1e-9 || !v.w.is_finite() {
            return None;
        }
        let (nx, ny) = (v.x / v.w, v.y / v.w);
        Some([((nx * 0.5 + 0.5) * f64::from(self.w)) as f32, ((0.5 - ny * 0.5) * f64::from(self.h)) as f32])
    }

    /// Pixel of a sketch point of this view's plane.
    #[must_use]
    pub fn to_screen(&self, p: [f64; 2]) -> Option<[f32; 2]> {
        self.project(&self.plane.to_3d(p))
    }

    /// The sketch point under pixel `(x, y)` (`None` when the ray misses the plane).
    #[must_use]
    pub fn to_plane(&self, x: f32, y: f32) -> Option<[f64; 2]> {
        let (eye, dir) = self.cam.ray(f64::from(x), f64::from(y), f64::from(self.w), f64::from(self.h));
        let n = self.plane.normal();
        let denom = n.dot(&dir);
        if denom.abs() < 1e-9 {
            return None;
        }
        let t = n.dot(&(Point3::from(self.plane.origin) - eye)) / denom;
        if t <= 0.0 || !t.is_finite() {
            return None;
        }
        Some(self.plane.to_2d(&(eye + dir * t)))
    }

    /// Sketch units per pixel at the middle of the viewport.
    #[must_use]
    pub fn units_per_px(&self) -> f64 {
        let (cx, cy) = (self.w / 2.0, self.h / 2.0);
        match (self.to_plane(cx, cy), self.to_plane(cx + 1.0, cy)) {
            (Some(a), Some(b)) => (a[0] - b[0]).hypot(a[1] - b[1]).max(1e-12),
            _ => 1.0,
        }
    }
}
