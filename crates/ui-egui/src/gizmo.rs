//! The manipulator gizmo: axis handles to translate or rotate an object in the viewport.
//!
//! The state machine is plain maths on a camera and pixel positions. A drag only *previews*
//! (the viewport draws the object moved by [`Gizmo::matrix`]); the drag ends with exactly one
//! matrix for one `mesh.transform` command (one undo step), or with nothing when it is cancelled.
use crate::select::View;
use surfacecraft_geom::{Point3, Transform, Vec3};

/// What the handles do.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum GizmoMode {
    /// Arrows move along an axis.
    #[default]
    Translate,
    /// Rings turn about an axis.
    Rotate,
}

/// A world axis.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Axis {
    /// +X.
    X,
    /// +Y.
    Y,
    /// +Z.
    Z,
}

impl Axis {
    /// All axes.
    pub const ALL: [Axis; 3] = [Axis::X, Axis::Y, Axis::Z];

    /// The unit direction.
    #[must_use]
    pub fn dir(self) -> Vec3 {
        match self {
            Axis::X => Vec3::x(),
            Axis::Y => Vec3::y(),
            Axis::Z => Vec3::z(),
        }
    }

    /// Two unit vectors that with the axis form a right-handed frame (`u × v = axis`).
    #[must_use]
    pub fn plane_basis(self) -> (Vec3, Vec3) {
        match self {
            Axis::X => (Vec3::y(), Vec3::z()),
            Axis::Y => (Vec3::z(), Vec3::x()),
            Axis::Z => (Vec3::x(), Vec3::y()),
        }
    }
}

/// Screen size of the gizmo's axes (px).
pub const HANDLE_PX: f64 = 90.0;
/// How close the pointer must be to a handle to grab it (px).
pub const GRAB_PX: f64 = 8.0;
/// Segments of a rotation ring.
pub const RING_SEGMENTS: usize = 48;

#[derive(Clone, Copy, Debug)]
struct Drag {
    axis: Axis,
    /// Translation: parameter along the axis at the start (world units); rotation: start angle.
    start: f64,
    /// The same quantity now.
    now: f64,
}

/// The gizmo of one object: where it stands, what it does and the drag in progress.
#[derive(Clone, Debug, Default)]
pub struct Gizmo {
    /// Translate or rotate.
    pub mode: GizmoMode,
    /// The pivot (the object's bounding-box centre when the drag began).
    pub center: Point3,
    drag: Option<Drag>,
}

fn wrap_pi(a: f64) -> f64 {
    let t = std::f64::consts::TAU;
    let mut r = (a + std::f64::consts::PI).rem_euclid(t) - std::f64::consts::PI;
    if r < -std::f64::consts::PI {
        r += t;
    }
    r
}

impl Gizmo {
    /// A gizmo in `mode` around `center`, with no drag.
    #[must_use]
    pub fn new(mode: GizmoMode, center: Point3) -> Self {
        Self { mode, center, drag: None }
    }

    /// Whether a drag is in progress.
    #[must_use]
    pub fn dragging(&self) -> bool {
        self.drag.is_some()
    }

    /// The axis being dragged.
    #[must_use]
    pub fn active_axis(&self) -> Option<Axis> {
        self.drag.map(|d| d.axis)
    }

    /// World length of a handle for the current view (constant on screen).
    #[must_use]
    pub fn handle_length(&self, view: &View<'_>) -> f64 {
        let (_, _, f) = view.camera.basis();
        let depth = (self.center - view.camera.eye()).dot(&f).max(view.camera.distance * 1e-3);
        view.camera.world_per_pixel(depth, view.h) * HANDLE_PX
    }

    /// The ring of an axis as world points (rotation mode).
    #[must_use]
    pub fn ring(&self, axis: Axis, radius: f64) -> Vec<Point3> {
        let (u, v) = axis.plane_basis();
        (0..=RING_SEGMENTS)
            .map(|i| {
                let a = std::f64::consts::TAU * i as f64 / RING_SEGMENTS as f64;
                self.center + (u * a.cos() + v * a.sin()) * radius
            })
            .collect()
    }

    /// The axis handle under pixel `cursor`, if any.
    #[must_use]
    pub fn handle_at(&self, view: &View<'_>, cursor: [f64; 2]) -> Option<Axis> {
        let len = self.handle_length(view);
        let c = view.camera.project(&self.center, view.w, view.h)?;
        let mut best: Option<(Axis, f64)> = None;
        for axis in Axis::ALL {
            let d = match self.mode {
                GizmoMode::Translate => {
                    let tip = view.camera.project(&(self.center + axis.dir() * len), view.w, view.h)?;
                    seg_dist(cursor, [c[0], c[1]], [tip[0], tip[1]])
                }
                GizmoMode::Rotate => {
                    let pts: Vec<[f64; 2]> = self
                        .ring(axis, len * 0.8)
                        .iter()
                        .filter_map(|p| view.camera.project(p, view.w, view.h))
                        .map(|p| [p[0], p[1]])
                        .collect();
                    pts.windows(2).map(|s| seg_dist(cursor, s[0], s[1])).fold(f64::INFINITY, f64::min)
                }
            };
            if d <= GRAB_PX && best.is_none_or(|(_, bd)| d < bd) {
                best = Some((axis, d));
            }
        }
        best.map(|(a, _)| a)
    }

    /// The drag quantity under `cursor`: the parameter along the axis (translate) or the angle
    /// about it (rotate); `None` where the pointer ray is parallel to what it is matched with.
    fn measure(&self, view: &View<'_>, axis: Axis, cursor: [f64; 2]) -> Option<f64> {
        let (o, d) = view.camera.ray(cursor[0], cursor[1], view.w, view.h);
        let a = axis.dir();
        match self.mode {
            GizmoMode::Translate => {
                // closest point of the axis line to the pointer ray
                let b = a.dot(&d);
                let den = 1.0 - b * b;
                if den < 1e-9 {
                    return None;
                }
                let r = self.center - o;
                let s = (b * d.dot(&r) - a.dot(&r)) / den;
                s.is_finite().then_some(s)
            }
            GizmoMode::Rotate => {
                // pointer ray against the rotation plane
                let den = d.dot(&a);
                if den.abs() < 1e-6 {
                    return None;
                }
                let t = (self.center - o).dot(&a) / den;
                let p = o + d * t - self.center;
                let (u, v) = axis.plane_basis();
                let ang = p.dot(&v).atan2(p.dot(&u));
                ang.is_finite().then_some(ang)
            }
        }
    }

    /// Starts a drag on `axis` at pixel `cursor`; false when the pointer is not on the object's
    /// handle plane (nothing to measure).
    pub fn begin(&mut self, view: &View<'_>, axis: Axis, cursor: [f64; 2]) -> bool {
        match self.measure(view, axis, cursor) {
            Some(m) => {
                self.drag = Some(Drag { axis, start: m, now: m });
                true
            }
            None => false,
        }
    }

    /// Moves the drag to pixel `cursor`.
    pub fn update(&mut self, view: &View<'_>, cursor: [f64; 2]) {
        if let Some(d) = self.drag
            && let Some(m) = self.measure(view, d.axis, cursor)
        {
            self.drag = Some(Drag { now: m, ..d });
        }
    }

    /// The motion so far (the identity without a drag).
    #[must_use]
    pub fn transform(&self) -> Transform {
        let Some(d) = self.drag else { return Transform::identity() };
        match self.mode {
            GizmoMode::Translate => Transform::translation(d.axis.dir() * (d.now - d.start)),
            GizmoMode::Rotate => {
                let c = self.center.coords;
                Transform::translation(c)
                    .then_after(&Transform::rotation_axis_angle(&d.axis.dir(), wrap_pi(d.now - d.start)))
                    .then_after(&Transform::translation(-c))
            }
        }
    }

    /// The motion so far as the row-major matrix of `mesh.transform`.
    #[must_use]
    pub fn matrix(&self) -> [[f64; 4]; 4] {
        self.transform().to_rows()
    }

    /// Ends the drag and returns its matrix, `None` when there was no drag or it moved nothing.
    pub fn finish(&mut self) -> Option<[[f64; 4]; 4]> {
        let d = self.drag.take()?;
        let moved = match self.mode {
            GizmoMode::Translate => (d.now - d.start).abs() > 1e-9,
            GizmoMode::Rotate => wrap_pi(d.now - d.start).abs() > 1e-9,
        };
        if !moved {
            return None;
        }
        self.drag = Some(d);
        let m = self.matrix();
        self.drag = None;
        Some(m)
    }

    /// Abandons the drag: nothing is applied.
    pub fn cancel(&mut self) {
        self.drag = None;
    }
}

fn seg_dist(p: [f64; 2], a: [f64; 2], b: [f64; 2]) -> f64 {
    let (dx, dy) = (b[0] - a[0], b[1] - a[1]);
    let l2 = dx * dx + dy * dy;
    let k = if l2 < 1e-12 { 0.0 } else { (((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / l2).clamp(0.0, 1.0) };
    (p[0] - (a[0] + dx * k)).hypot(p[1] - (a[1] + dy * k))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::camera::{Camera, StandardView};

    const W: f64 = 400.0;
    const H: f64 = 300.0;

    fn setup(view: StandardView) -> Camera {
        let mut c = Camera { target: Point3::new(1.0, 2.0, 3.0), distance: 60.0, ..Camera::default() };
        c.set_standard(view);
        c
    }

    fn px(cam: &Camera, p: Point3) -> [f64; 2] {
        let s = cam.project(&p, W, H).unwrap();
        [s[0], s[1]]
    }

    fn point(m: &[[f64; 4]; 4], p: Point3) -> Point3 {
        Transform::from_rows(m).unwrap().apply(&p)
    }

    /// Dragging the X handle along the X axis gives a pure +X translation of the distance dragged.
    #[test]
    fn translate_drag_gives_the_translation_along_the_axis() {
        let cam = setup(StandardView::Front);
        let v = View { camera: &cam, w: W, h: H };
        let mut g = Gizmo { mode: GizmoMode::Translate, center: cam.target, ..Gizmo::default() };
        let len = g.handle_length(&v);
        assert!(len > 1.0);
        // the handle is found where it is drawn
        let on = px(&cam, g.center + Vec3::x() * len * 0.5);
        assert_eq!(g.handle_at(&v, [on[0], on[1] + 3.0]), Some(Axis::X));
        assert_eq!(g.handle_at(&v, [on[0], on[1] + 60.0]), None);
        assert!(g.begin(&v, Axis::X, on));
        // pointer moves to where the point 7 units further along X is drawn
        g.update(&v, px(&cam, g.center + Vec3::x() * (len * 0.5 + 7.0)));
        let m = g.finish().expect("moved");
        assert!(!g.dragging());
        let moved = point(&m, Point3::new(10.0, 20.0, 30.0));
        assert!((moved - Point3::new(17.0, 20.0, 30.0)).norm() < 1e-6, "{moved:?}");
        // the rotation block is the identity
        assert!((m[0][0] - 1.0).abs() < 1e-12 && m[0][1].abs() < 1e-12 && (m[2][2] - 1.0).abs() < 1e-12);
    }

    /// An axis the view looks along cannot be measured: begin refuses instead of dividing by ~0.
    #[test]
    fn dragging_along_the_view_direction_is_refused() {
        let cam = setup(StandardView::Front);
        let v = View { camera: &cam, w: W, h: H };
        let mut g = Gizmo { center: cam.target, ..Gizmo::default() };
        assert!(!g.begin(&v, Axis::Y, [W / 2.0, H / 2.0]), "the front view looks along Y");
        assert!(g.finish().is_none());
    }

    /// A quarter-turn drag about Z rotates 90° about the pivot and keeps the pivot fixed.
    #[test]
    fn rotate_drag_gives_the_rotation_about_the_pivot() {
        let cam = setup(StandardView::Top);
        let v = View { camera: &cam, w: W, h: H };
        let mut g = Gizmo { mode: GizmoMode::Rotate, center: cam.target, ..Gizmo::default() };
        let r = g.handle_length(&v) * 0.8;
        let c = g.center;
        let on_ring = |a: f64| px(&cam, c + (Vec3::x() * a.cos() + Vec3::y() * a.sin()) * r);
        assert_eq!(g.handle_at(&v, on_ring(0.7)), Some(Axis::Z));
        assert!(g.begin(&v, Axis::Z, on_ring(0.0)));
        g.update(&v, on_ring(std::f64::consts::FRAC_PI_2));
        let m = g.finish().expect("turned");
        // the pivot stays; a point at +X of the pivot goes to +Y of it
        assert!((point(&m, g.center) - g.center).norm() < 1e-6);
        let p = point(&m, g.center + Vec3::x() * 5.0);
        assert!((p - (g.center + Vec3::y() * 5.0)).norm() < 1e-3, "{p:?}");
    }

    /// Cancelling (Esc) ends the drag without a result, and the preview matrix is the identity.
    #[test]
    fn cancel_leaves_nothing_to_apply() {
        let cam = setup(StandardView::Front);
        let v = View { camera: &cam, w: W, h: H };
        let mut g = Gizmo { center: cam.target, ..Gizmo::default() };
        let on = px(&cam, g.center + Vec3::x() * 5.0);
        assert!(g.begin(&v, Axis::X, on));
        g.update(&v, [on[0] + 40.0, on[1]]);
        assert!(g.matrix()[0][3].abs() > 1.0, "the preview moves");
        g.cancel();
        assert!(!g.dragging() && g.finish().is_none());
        assert_eq!(g.matrix(), Transform::identity().to_rows());
        // a click without movement is not a command either
        assert!(g.begin(&v, Axis::X, on));
        assert!(g.finish().is_none());
    }

    #[test]
    fn wrap_keeps_angles_in_half_a_turn() {
        for a in [-7.0, -3.5, 0.0, 3.2, 6.5, 12.0] {
            let w = wrap_pi(a);
            assert!((-std::f64::consts::PI..=std::f64::consts::PI).contains(&w), "{a} -> {w}");
        }
    }
}
