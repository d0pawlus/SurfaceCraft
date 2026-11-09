//! The view cube: a small orientation cube in a corner of the viewport whose faces turn the view
//! to the standard views (front, back, left, right, top, bottom). Plain geometry here (testable
//! without egui); [`crate::viewtools`] paints it and routes the clicks.
use crate::camera::{Camera, StandardView};
use surfacecraft_geom::Vec3;

/// Faces whose normal is within this of edge-on are not drawn or clicked.
const EDGE_ON: f64 = 0.02;

/// One visible face of the cube on screen.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CubeFace {
    /// The view this face turns to.
    pub view: StandardView,
    /// Corners in screen pixels.
    pub quad: [[f64; 2]; 4],
    /// Depth of the face centre (larger = farther).
    pub depth: f64,
}

/// The faces of a cube drawn at `centre` with half-size `half` px as the camera sees it
/// (orthographic, orientation only), farthest first so painting in order is correct.
#[must_use]
pub fn visible_faces(camera: &Camera, centre: [f64; 2], half: f64) -> Vec<CubeFace> {
    let (right, up, fwd) = camera.basis();
    let s = half * 0.5;
    let screen = |p: Vec3| [centre[0] + p.dot(&right) * s, centre[1] - p.dot(&up) * s];
    let mut faces = Vec::new();
    for view in StandardView::ALL {
        let Some(n) = view.face_normal() else { continue };
        if n.dot(&fwd) >= -EDGE_ON {
            continue;
        }
        // the two other axes span the face
        let (u, v) = if n.x.abs() > 0.5 {
            (Vec3::y(), Vec3::z())
        } else if n.y.abs() > 0.5 {
            (Vec3::x(), Vec3::z())
        } else {
            (Vec3::x(), Vec3::y())
        };
        let corner = |a: f64, b: f64| screen(n + u * a + v * b);
        faces.push(CubeFace {
            view,
            quad: [corner(-1.0, -1.0), corner(1.0, -1.0), corner(1.0, 1.0), corner(-1.0, 1.0)],
            depth: n.dot(&fwd),
        });
    }
    // facing the eye most directly = smallest dot = nearest = last
    faces.sort_by(|a, b| b.depth.total_cmp(&a.depth));
    faces
}

fn inside_quad(q: &[[f64; 2]; 4], p: [f64; 2]) -> bool {
    let mut sign = 0.0;
    for i in 0..4 {
        let (a, b) = (q[i], q[(i + 1) % 4]);
        let cross = (b[0] - a[0]) * (p[1] - a[1]) - (b[1] - a[1]) * (p[0] - a[0]);
        if cross.abs() < 1e-12 {
            continue;
        }
        if sign == 0.0 {
            sign = cross.signum();
        } else if cross.signum() != sign {
            return false;
        }
    }
    sign != 0.0
}

/// The view of the face under `p` (nearest face first).
#[must_use]
pub fn face_at(faces: &[CubeFace], p: [f64; 2]) -> Option<StandardView> {
    faces.iter().rev().find(|f| inside_quad(&f.quad, p)).map(|f| f.view)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cam(view: StandardView) -> Camera {
        let mut c = Camera::default();
        c.set_standard(view);
        c
    }

    /// Facing a standard view, the cube shows exactly that face, centred, and clicking it asks for
    /// the same view; the other faces are edge-on.
    #[test]
    fn a_standard_view_shows_its_own_face_in_the_middle() {
        for view in [
            StandardView::Front,
            StandardView::Back,
            StandardView::Left,
            StandardView::Right,
            StandardView::Top,
            StandardView::Bottom,
        ] {
            let faces = visible_faces(&cam(view), [100.0, 100.0], 40.0);
            assert_eq!(faces.len(), 1, "{view:?}");
            assert_eq!(faces[0].view, view);
            assert_eq!(face_at(&faces, [100.0, 100.0]), Some(view));
            assert_eq!(face_at(&faces, [300.0, 100.0]), None);
        }
    }

    /// From the isometric view three faces are visible (front, right, top), the nearest on top, and
    /// each region of the cube maps to a different face.
    #[test]
    fn the_iso_view_shows_front_right_and_top() {
        let faces = visible_faces(&cam(StandardView::Iso), [100.0, 100.0], 40.0);
        let mut views: Vec<_> = faces.iter().map(|f| format!("{:?}", f.view)).collect();
        views.sort();
        assert_eq!(views, ["Front", "Right", "Top"]);
        // the top face is above the centre of the drawing, front to the lower left, right to the lower right
        let centre_of = |v: StandardView| {
            let q = faces.iter().find(|f| f.view == v).unwrap().quad;
            [(q[0][0] + q[2][0]) / 2.0, (q[0][1] + q[2][1]) / 2.0]
        };
        let (top, front, right) =
            (centre_of(StandardView::Top), centre_of(StandardView::Front), centre_of(StandardView::Right));
        assert!(top[1] < front[1] && top[1] < right[1], "top above: {top:?} {front:?} {right:?}");
        assert!(front[0] < right[0], "front left of right");
        for v in [StandardView::Top, StandardView::Front, StandardView::Right] {
            assert_eq!(face_at(&faces, centre_of(v)), Some(v));
        }
    }
}
