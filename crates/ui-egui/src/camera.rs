//! The one orbit camera of the viewport.
use surfacecraft_geom::{Aabb, Mat4, Point3, Vec3};

/// Orbit camera around a target point (Z up).
#[derive(Clone, Debug, PartialEq)]
pub struct Camera {
    /// Point orbited.
    pub target: Point3,
    /// Distance from target to eye.
    pub distance: f64,
    /// Azimuth (radians, around Z).
    pub yaw: f64,
    /// Elevation (radians).
    pub pitch: f64,
    /// Vertical field of view (radians).
    pub fov: f64,
}

impl Default for Camera {
    fn default() -> Self {
        Self { target: Point3::origin(), distance: 150.0, yaw: -0.8, pitch: 0.5, fov: 0.7 }
    }
}

impl Camera {
    /// Eye position.
    #[must_use]
    pub fn eye(&self) -> Point3 {
        let (sy, cy) = self.yaw.sin_cos();
        let (sp, cp) = self.pitch.sin_cos();
        self.target + Vec3::new(cp * cy, cp * sy, sp) * self.distance
    }

    /// Unit view direction (eye → target).
    #[must_use]
    pub fn forward(&self) -> Vec3 {
        surfacecraft_geom::normalize(self.target - self.eye()).unwrap_or_else(|| -Vec3::z())
    }

    /// View matrix (world → camera), relative to `origin` (shifted for f32 precision).
    #[must_use]
    pub fn view(&self, origin: &Point3) -> Mat4 {
        let eye = self.eye() - origin.coords;
        let target = self.target - origin.coords;
        Mat4::look_at_rh(&eye, &target, &Vec3::z())
    }

    /// Projection for `aspect` = width / height.
    #[must_use]
    pub fn projection(&self, aspect: f64) -> Mat4 {
        let near = (self.distance * 0.002).max(1e-4);
        let far = self.distance * 50.0 + 10.0;
        // wgpu clip space z in [0, 1]
        let f = 1.0 / (self.fov / 2.0).tan();
        let a = if aspect.is_finite() && aspect > 0.0 { aspect } else { 1.0 };
        Mat4::new(
            f / a,
            0.0,
            0.0,
            0.0,
            0.0,
            f,
            0.0,
            0.0,
            0.0,
            0.0,
            far / (near - far),
            far * near / (near - far),
            0.0,
            0.0,
            -1.0,
            0.0,
        )
    }

    /// Frames `bbox` (no change when it is empty).
    pub fn fit(&mut self, bbox: &Aabb) {
        if bbox.is_empty() {
            return;
        }
        self.target = bbox.center();
        let r = (bbox.diagonal() * 0.5).max(1e-3);
        self.distance = r / (self.fov / 2.0).sin() * 1.1;
    }

    /// Orbits by a screen drag (pixels).
    pub fn orbit(&mut self, dx: f64, dy: f64) {
        self.yaw -= dx * 0.008;
        self.pitch = (self.pitch + dy * 0.008).clamp(-1.55, 1.55);
    }

    /// Pans by a screen drag (pixels) for a viewport `height` px tall.
    pub fn pan(&mut self, dx: f64, dy: f64, height: f64) {
        let f = self.forward();
        let right = surfacecraft_geom::normalize(f.cross(&Vec3::z())).unwrap_or_else(Vec3::x);
        let up = right.cross(&f);
        let scale = 2.0 * self.distance * (self.fov / 2.0).tan() / height.max(1.0);
        self.target += (-right * dx + up * dy) * scale;
    }

    /// Zooms by scroll `amount` (positive = closer).
    pub fn zoom(&mut self, amount: f64) {
        self.distance = (self.distance * (-amount * 0.002).exp()).clamp(1e-3, 1e7);
    }

    /// The world-space ray through viewport pixel `(x, y)` of a `w × h` viewport.
    #[must_use]
    pub fn ray(&self, x: f64, y: f64, w: f64, h: f64) -> (Point3, Vec3) {
        let ndc_x = 2.0 * x / w.max(1.0) - 1.0;
        let ndc_y = 1.0 - 2.0 * y / h.max(1.0);
        let t = (self.fov / 2.0).tan();
        let f = self.forward();
        let right = surfacecraft_geom::normalize(f.cross(&Vec3::z())).unwrap_or_else(Vec3::x);
        let up = right.cross(&f);
        let dir = f + right * (ndc_x * t * w / h.max(1.0)) + up * (ndc_y * t);
        (self.eye(), surfacecraft_geom::normalize(dir).unwrap_or(f))
    }

    /// The orthonormal screen basis `(right, up, forward)` of the view.
    #[must_use]
    pub fn basis(&self) -> (Vec3, Vec3, Vec3) {
        let f = self.forward();
        let right = surfacecraft_geom::normalize(f.cross(&Vec3::z())).unwrap_or_else(Vec3::x);
        let up = right.cross(&f);
        (right, up, f)
    }

    /// Projects world point `p` to pixels of a `w × h` viewport: `[x, y, depth]` with `y`
    /// downwards and `depth` the distance along the view direction. `None` at or behind the eye.
    #[must_use]
    pub fn project(&self, p: &Point3, w: f64, h: f64) -> Option<[f64; 3]> {
        let (right, up, f) = self.basis();
        let v = p - self.eye();
        let depth = v.dot(&f);
        if depth <= self.distance * 1e-4 {
            return None;
        }
        let t = (self.fov / 2.0).tan();
        let ndc_x = v.dot(&right) / (depth * t * w / h.max(1.0));
        let ndc_y = v.dot(&up) / (depth * t);
        Some([(ndc_x + 1.0) * 0.5 * w, (1.0 - ndc_y) * 0.5 * h, depth])
    }

    /// The distance of `p` along the view direction from the eye: positive in front, negative
    /// behind it.
    #[must_use]
    pub fn depth(&self, p: &Point3) -> f64 {
        let (_, _, f) = self.basis();
        (p - self.eye()).dot(&f)
    }

    /// World length of one screen pixel at distance `depth` for a viewport `h` px tall.
    #[must_use]
    pub fn world_per_pixel(&self, depth: f64, h: f64) -> f64 {
        2.0 * depth * (self.fov / 2.0).tan() / h.max(1.0)
    }

    /// Turns the view to a standard orientation, keeping target, distance and field of view.
    pub fn set_standard(&mut self, view: StandardView) {
        let (yaw, pitch) = view.angles();
        self.yaw = yaw;
        self.pitch = pitch;
    }
}

/// Pitch of the top and bottom views: just short of the pole, where the Z-up look-at is undefined.
pub const POLE_PITCH: f64 = std::f64::consts::FRAC_PI_2 - 1e-3;

/// The standard views of the view cube.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum StandardView {
    /// Looking along +Y (eye at -Y).
    Front,
    /// Looking along -Y (eye at +Y).
    Back,
    /// Looking along +X (eye at -X).
    Left,
    /// Looking along -X (eye at +X).
    Right,
    /// Looking down -Z.
    Top,
    /// Looking up +Z.
    Bottom,
    /// Isometric: from front, right and above.
    Iso,
}

impl StandardView {
    /// Every view, in menu order.
    pub const ALL: [StandardView; 7] = [
        StandardView::Front,
        StandardView::Back,
        StandardView::Left,
        StandardView::Right,
        StandardView::Top,
        StandardView::Bottom,
        StandardView::Iso,
    ];

    /// The catalogue key of the view's name.
    #[must_use]
    pub fn key(self) -> &'static str {
        match self {
            StandardView::Front => "view.front",
            StandardView::Back => "view.back",
            StandardView::Left => "view.left",
            StandardView::Right => "view.right",
            StandardView::Top => "view.top",
            StandardView::Bottom => "view.bottom",
            StandardView::Iso => "view.iso",
        }
    }

    /// `(yaw, pitch)` of the orbit camera for this view.
    #[must_use]
    pub fn angles(self) -> (f64, f64) {
        use std::f64::consts::{FRAC_PI_2, FRAC_PI_4, PI};
        match self {
            StandardView::Front => (-FRAC_PI_2, 0.0),
            StandardView::Back => (FRAC_PI_2, 0.0),
            StandardView::Left => (PI, 0.0),
            StandardView::Right => (0.0, 0.0),
            StandardView::Top => (-FRAC_PI_2, POLE_PITCH),
            StandardView::Bottom => (-FRAC_PI_2, -POLE_PITCH),
            StandardView::Iso => (-FRAC_PI_4, (1.0f64 / 2.0f64.sqrt()).atan()),
        }
    }

    /// Outward unit normal of this view's cube face (the direction to the eye); `None` for the
    /// isometric view.
    #[must_use]
    pub fn face_normal(self) -> Option<Vec3> {
        match self {
            StandardView::Front => Some(-Vec3::y()),
            StandardView::Back => Some(Vec3::y()),
            StandardView::Left => Some(-Vec3::x()),
            StandardView::Right => Some(Vec3::x()),
            StandardView::Top => Some(Vec3::z()),
            StandardView::Bottom => Some(-Vec3::z()),
            StandardView::Iso => None,
        }
    }
}

/// An animated change of orientation (yaw and pitch), the short way round.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ViewAnim {
    from: (f64, f64),
    to: (f64, f64),
    /// Seconds elapsed.
    pub t: f64,
}

/// Seconds a view change takes.
pub const ANIM_SECONDS: f64 = 0.25;

impl ViewAnim {
    /// An animation from the camera's current angles to `view`.
    #[must_use]
    pub fn to_view(cam: &Camera, view: StandardView) -> Self {
        let (ty, tp) = view.angles();
        let mut dy = (ty - cam.yaw).rem_euclid(std::f64::consts::TAU);
        if dy > std::f64::consts::PI {
            dy -= std::f64::consts::TAU;
        }
        Self { from: (cam.yaw, cam.pitch), to: (cam.yaw + dy, tp), t: 0.0 }
    }

    /// Advances by `dt` seconds and applies the angles to `cam`; true while still running.
    pub fn step(&mut self, cam: &mut Camera, dt: f64) -> bool {
        self.t += dt.max(0.0);
        let k = (self.t / ANIM_SECONDS).clamp(0.0, 1.0);
        let e = k * k * (3.0 - 2.0 * k);
        cam.yaw = self.from.0 + (self.to.0 - self.from.0) * e;
        cam.pitch = self.from.1 + (self.to.1 - self.from.1) * e;
        k < 1.0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Depth is the distance along the view direction: the target lies one distance in front of the
    /// eye, and a point behind the eye has a negative depth.
    #[test]
    fn depth_runs_along_the_view_direction_from_the_eye() {
        let cam = Camera::default();
        let eye = cam.eye();
        let (_, _, f) = cam.basis();
        assert!((cam.depth(&cam.target) - cam.distance).abs() < 1e-9);
        assert!((cam.depth(&(eye + f * 10.0)) - 10.0).abs() < 1e-9);
        assert!((cam.depth(&(eye - f * 3.0)) + 3.0).abs() < 1e-9);
    }

    fn near(a: Vec3, b: Vec3) -> bool {
        (a - b).norm() < 2e-3
    }

    /// Each standard view looks the way its name says, with the right axis to the right and the
    /// up axis up on the screen.
    #[test]
    fn standard_views_have_the_conventional_orientations() {
        // (view, forward, screen right, screen up)
        let cases = [
            (StandardView::Front, Vec3::y(), Vec3::x(), Vec3::z()),
            (StandardView::Back, -Vec3::y(), -Vec3::x(), Vec3::z()),
            (StandardView::Right, -Vec3::x(), Vec3::y(), Vec3::z()),
            (StandardView::Left, Vec3::x(), -Vec3::y(), Vec3::z()),
            (StandardView::Top, -Vec3::z(), Vec3::x(), Vec3::y()),
            (StandardView::Bottom, Vec3::z(), Vec3::x(), -Vec3::y()),
        ];
        for (view, f, r, u) in cases {
            let mut c = Camera::default();
            c.set_standard(view);
            let (right, up, fwd) = c.basis();
            assert!(near(fwd, f), "{view:?} forward {fwd:?}");
            assert!(near(right, r), "{view:?} right {right:?}");
            assert!(near(up, u), "{view:?} up {up:?}");
            // the cube face of the view faces the eye
            let n = view.face_normal().unwrap();
            assert!(near(n, -f), "{view:?} face normal");
        }
        let mut c = Camera::default();
        c.set_standard(StandardView::Iso);
        let (_, up, fwd) = c.basis();
        assert!(near(fwd, -Vec3::new(1.0, -1.0, 1.0).normalize()), "iso forward {fwd:?}");
        assert!(up.z > 0.5);
    }

    #[test]
    fn project_inverts_ray_and_view_animation_takes_the_short_way() {
        let mut c = Camera::default();
        c.fit(&Aabb::from_points(&[Point3::origin(), Point3::new(10.0, 20.0, 30.0)]));
        let (o, d) = c.ray(30.0, 70.0, 100.0, 80.0);
        let p = o + d * c.distance;
        let s = c.project(&p, 100.0, 80.0).unwrap();
        assert!((s[0] - 30.0).abs() < 1e-6 && (s[1] - 70.0).abs() < 1e-6, "{s:?}");
        assert!(c.project(&(c.eye() - c.forward() * 5.0), 100.0, 80.0).is_none());
        // from yaw 3.0 to the left view (yaw π) is 0.14 rad, not a lap
        let mut cam = Camera { yaw: 3.0, pitch: 0.3, ..Camera::default() };
        let mut a = ViewAnim::to_view(&cam, StandardView::Left);
        assert!(a.step(&mut cam, 0.01));
        assert!((cam.yaw - 3.0).abs() < 0.2);
        assert!(!a.step(&mut cam, 1.0));
        assert!((cam.yaw - std::f64::consts::PI).abs() < 1e-9 && cam.pitch.abs() < 1e-9);
    }

    #[test]
    fn centre_ray_hits_target_and_projection_maps_target_to_centre() {
        let mut c = Camera::default();
        c.fit(&Aabb::from_points(&[Point3::new(0.0, 0.0, 0.0), Point3::new(10.0, 20.0, 30.0)]));
        let (o, d) = c.ray(50.0, 40.0, 100.0, 80.0);
        let to_target = (c.target - o).normalize();
        assert!((d - to_target).norm() < 1e-9);
        let origin = Point3::new(1.0, 1.0, 1.0);
        let m = c.projection(1.25) * c.view(&origin);
        let p = m * (c.target - origin.coords).to_homogeneous();
        assert!((p.x / p.w).abs() < 1e-9 && (p.y / p.w).abs() < 1e-9);
        assert!((0.0..1.0).contains(&(p.z / p.w)));
    }
}
