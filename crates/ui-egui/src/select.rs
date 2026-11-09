//! Triangle selection on meshes: brush, lasso and smart select, in screen space.
//!
//! Everything here is plain maths on a mesh and a camera (no GPU, no egui), so it is testable
//! with a synthetic mesh and a known camera. A selection always speaks of the triangle ids of the
//! *full-resolution* mesh: when the display uses a coarser level of detail (`geom::lod`), callers
//! pass the full mesh (`DisplayItem::pick_mesh`) and map the ids to drawn triangles only for
//! highlighting (`DisplayItem::drawn_triangles`).
use crate::camera::Camera;
use std::collections::BTreeSet;
use surfacecraft_geom::{Bvh, Mesh, Point3};

/// A camera looking into a `w × h` pixel viewport.
#[derive(Clone, Copy, Debug)]
pub struct View<'a> {
    /// The camera.
    pub camera: &'a Camera,
    /// Viewport width (px).
    pub w: f64,
    /// Viewport height (px).
    pub h: f64,
}

/// How a new triangle set combines with the selection already there.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum SelectOp {
    /// The new set replaces the selection.
    #[default]
    Replace,
    /// The new set is added.
    Add,
    /// The new set is removed.
    Subtract,
}

/// A set of triangles of one mesh (original, full-resolution ids).
#[derive(Clone, Debug, Default, PartialEq)]
pub struct TriSelection {
    /// The mesh object the ids belong to.
    pub object: Option<surfacecraft_doc::ObjectId>,
    /// The selected triangles.
    pub tris: BTreeSet<u32>,
}

impl TriSelection {
    /// Combines `new` with the selection.
    pub fn apply(&mut self, op: SelectOp, new: &[u32]) {
        match op {
            SelectOp::Replace => self.tris = new.iter().copied().collect(),
            SelectOp::Add => self.tris.extend(new.iter().copied()),
            SelectOp::Subtract => {
                for t in new {
                    self.tris.remove(t);
                }
            }
        }
    }

    /// The triangles, ascending.
    #[must_use]
    pub fn to_vec(&self) -> Vec<u32> {
        self.tris.iter().copied().collect()
    }

    /// Whether nothing is selected.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.tris.is_empty()
    }
}

/// Which triangles count as visible to a screen-space tool.
#[derive(Clone, Copy)]
pub struct Visibility<'a> {
    /// Through-selection: back-facing and hidden triangles are selected too.
    pub through: bool,
    /// Hit-tester of the mesh, to tell hidden triangles from visible ones (front-facing ones
    /// that another part of the mesh covers). Without it only the facing test applies.
    pub bvh: Option<&'a Bvh>,
}

fn centroid(mesh: &Mesh, t: usize) -> Option<Point3> {
    let [a, b, c] = mesh.triangles.get(t)?;
    let (pa, pb, pc) =
        (mesh.positions.get(*a as usize)?, mesh.positions.get(*b as usize)?, mesh.positions.get(*c as usize)?);
    Some(Point3::from((pa.coords + pb.coords + pc.coords) / 3.0))
}

/// Triangles whose centroid projects to a pixel for which `inside` is true (checked inside the
/// pixel box `bounds = [min_x, min_y, max_x, max_y]` first), keeping only the triangles that face
/// the eye and that nothing covers unless `vis.through`.
#[must_use]
pub fn select_in_screen(
    mesh: &Mesh,
    view: &View<'_>,
    bounds: [f64; 4],
    vis: Visibility<'_>,
    inside: &dyn Fn(f64, f64) -> bool,
) -> Vec<u32> {
    let eye = view.camera.eye();
    let mut out = Vec::new();
    for t in 0..mesh.triangles.len() {
        let Some(c) = centroid(mesh, t) else { continue };
        let Some([x, y, _]) = view.camera.project(&c, view.w, view.h) else { continue };
        if x < bounds[0] || x > bounds[2] || y < bounds[1] || y > bounds[3] || !inside(x, y) {
            continue;
        }
        if !vis.through {
            let Some(n) = mesh.face_normal(t) else { continue };
            let to_eye = eye - c;
            if n.dot(&to_eye) <= 0.0 {
                continue;
            }
            if let Some(bvh) = vis.bvh {
                let dist = to_eye.norm();
                if let Some(dir) = surfacecraft_geom::normalize(c - eye)
                    && let Some((hit, hit_t)) = bvh.ray_cast(&eye, &dir)
                    && hit != t
                    && hit_t < dist - dist * 1e-6 - 1e-9
                {
                    continue;
                }
            }
        }
        out.push(t as u32);
    }
    out
}

/// Even-odd point-in-polygon test (pixel coordinates).
#[must_use]
pub fn point_in_polygon(poly: &[[f64; 2]], x: f64, y: f64) -> bool {
    let mut inside = false;
    let n = poly.len();
    for i in 0..n {
        let (Some(a), Some(b)) = (poly.get(i), poly.get((i + 1) % n)) else { continue };
        if (a[1] > y) != (b[1] > y) && x < (b[0] - a[0]) * (y - a[1]) / (b[1] - a[1]) + a[0] {
            inside = !inside;
        }
    }
    inside
}

/// Lasso: triangles under the screen polygon `poly` (at least three points).
#[must_use]
pub fn lasso_select(mesh: &Mesh, view: &View<'_>, poly: &[[f64; 2]], vis: Visibility<'_>) -> Vec<u32> {
    if poly.len() < 3 || poly.iter().any(|p| !p[0].is_finite() || !p[1].is_finite()) {
        return Vec::new();
    }
    let mut b = [f64::INFINITY, f64::INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY];
    for p in poly {
        b = [b[0].min(p[0]), b[1].min(p[1]), b[2].max(p[0]), b[3].max(p[1])];
    }
    select_in_screen(mesh, view, b, vis, &|x, y| point_in_polygon(poly, x, y))
}

/// Brush: triangles within `radius` px of any of the `centres` (a stroke sampled by
/// [`stroke_points`]).
#[must_use]
pub fn brush_select(mesh: &Mesh, view: &View<'_>, centres: &[[f64; 2]], radius: f64, vis: Visibility<'_>) -> Vec<u32> {
    let radius = radius.clamp(1.0, 2000.0);
    if centres.is_empty() || centres.iter().any(|p| !p[0].is_finite() || !p[1].is_finite()) {
        return Vec::new();
    }
    let mut b = [f64::INFINITY, f64::INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY];
    for p in centres {
        b = [b[0].min(p[0] - radius), b[1].min(p[1] - radius), b[2].max(p[0] + radius), b[3].max(p[1] + radius)];
    }
    let r2 = radius * radius;
    select_in_screen(mesh, view, b, vis, &|x, y| centres.iter().any(|c| (c[0] - x).powi(2) + (c[1] - y).powi(2) <= r2))
}

/// Points along the pointer movement `from → to` at most `radius / 2` apart, so a fast stroke
/// leaves no gaps (both ends included).
#[must_use]
pub fn stroke_points(from: [f64; 2], to: [f64; 2], radius: f64) -> Vec<[f64; 2]> {
    let d = ((to[0] - from[0]).powi(2) + (to[1] - from[1]).powi(2)).sqrt();
    if !d.is_finite() {
        return Vec::new();
    }
    let n = ((d / (radius.max(1.0) * 0.5)).ceil() as usize).clamp(1, 4096);
    (0..=n)
        .map(|i| {
            let k = i as f64 / n as f64;
            [from[0] + (to[0] - from[0]) * k, from[1] + (to[1] - from[1]) * k]
        })
        .collect()
}

/// Smart select: the triangles reachable from `seed` without crossing an edge whose dihedral
/// angle exceeds `max_angle_deg` (the engine's `region.grow` uses the same growth).
#[must_use]
pub fn smart_select(mesh: &Mesh, seed: u32, max_angle_deg: f64) -> Vec<u32> {
    if !(max_angle_deg.is_finite() && max_angle_deg > 0.0 && max_angle_deg < 180.0) {
        return Vec::new();
    }
    surfacecraft_fit::segment::grow_smooth(mesh, &[seed], max_angle_deg)
}

#[cfg(test)]
mod tests {
    use super::*;
    use surfacecraft_geom::synth;

    /// A camera two diagonals in front of the mesh along -Y, looking at its centre.
    fn front_camera(mesh: &Mesh) -> Camera {
        let mut c = Camera::default();
        c.fit(&mesh.bbox());
        c.set_standard(crate::camera::StandardView::Front);
        c
    }

    const W: f64 = 400.0;
    const H: f64 = 300.0;

    fn view(c: &Camera) -> View<'_> {
        View { camera: c, w: W, h: H }
    }

    fn direct() -> Visibility<'static> {
        Visibility { through: false, bvh: None }
    }

    #[test]
    fn polygon_test_is_even_odd() {
        let sq = [[0.0, 0.0], [10.0, 0.0], [10.0, 10.0], [0.0, 10.0]];
        assert!(point_in_polygon(&sq, 5.0, 5.0));
        assert!(!point_in_polygon(&sq, 15.0, 5.0));
        let concave = [[0.0, 0.0], [10.0, 0.0], [10.0, 10.0], [5.0, 3.0], [0.0, 10.0]];
        assert!(!point_in_polygon(&concave, 5.0, 8.0), "in the notch");
        assert!(point_in_polygon(&concave, 5.0, 1.0));
    }

    /// A lasso round the whole screen takes exactly the triangles that face the camera.
    #[test]
    fn lasso_takes_front_facing_triangles_only_unless_through() {
        // closed sphere, faces towards +Y (the eye is at -Y)
        let mesh = synth::sphere(Point3::origin(), 10.0, 24, 12);
        let cam = front_camera(&mesh);
        let all = [[0.0, 0.0], [W, 0.0], [W, H], [0.0, H]];
        let front = lasso_select(&mesh, &view(&cam), &all, direct());
        let through = lasso_select(&mesh, &view(&cam), &all, Visibility { through: true, bvh: None });
        assert_eq!(through.len(), mesh.triangle_count(), "through takes everything on screen");
        assert!(!front.is_empty() && front.len() < mesh.triangle_count());
        // every selected triangle looks at the eye, and the facing ones are all there
        let eye = cam.eye();
        for t in 0..mesh.triangle_count() {
            let facing = mesh.face_normal(t).unwrap().dot(&(eye - centroid(&mesh, t).unwrap())) > 0.0;
            assert_eq!(front.contains(&(t as u32)), facing, "triangle {t}");
        }
        // roughly half of a sphere faces the eye
        let ratio = front.len() as f64 / mesh.triangle_count() as f64;
        assert!((0.35..0.65).contains(&ratio), "{ratio}");
    }

    /// A small lasso in the middle takes a patch around the centre only.
    #[test]
    fn lasso_polygon_limits_the_selection_to_its_area() {
        let mesh = synth::sphere(Point3::origin(), 10.0, 48, 24);
        let cam = front_camera(&mesh);
        let (cx, cy) = (W / 2.0, H / 2.0);
        let poly = [[cx - 30.0, cy - 30.0], [cx + 30.0, cy - 30.0], [cx + 30.0, cy + 30.0], [cx - 30.0, cy + 30.0]];
        let sel = lasso_select(&mesh, &view(&cam), &poly, direct());
        assert!(!sel.is_empty());
        for &t in &sel {
            let p = cam.project(&centroid(&mesh, t as usize).unwrap(), W, H).unwrap();
            assert!((p[0] - cx).abs() <= 30.0 && (p[1] - cy).abs() <= 30.0);
        }
        let big = [[0.0, 0.0], [W, 0.0], [W, H], [0.0, H]];
        assert!(sel.len() < lasso_select(&mesh, &view(&cam), &big, direct()).len() / 2);
        // a degenerate polygon selects nothing
        assert!(lasso_select(&mesh, &view(&cam), &poly[..2], direct()).is_empty());
    }

    /// A triangle behind another is hidden unless the selection goes through.
    #[test]
    fn hidden_triangles_need_through_selection() {
        // two parallel plates facing +Y... both face the eye, the rear one is covered
        let mut mesh = Mesh::default();
        for (y, x0) in [(0.0, 0.0), (5.0, 0.0)] {
            let b = mesh.positions.len() as u32;
            // counter-clockwise seen from -Y: normal towards -Y
            mesh.positions.push(Point3::new(x0, y, 0.0));
            mesh.positions.push(Point3::new(x0 + 10.0, y, 0.0));
            mesh.positions.push(Point3::new(x0, y, 10.0));
            mesh.triangles.push([b, b + 1, b + 2]);
        }
        let n0 = mesh.face_normal(0).unwrap();
        assert!(n0.y < 0.0, "normal towards the eye at -Y: {n0:?}");
        let bvh = Bvh::build(&mesh);
        let cam = front_camera(&mesh);
        let all = [[0.0, 0.0], [W, 0.0], [W, H], [0.0, H]];
        let direct_sel = lasso_select(&mesh, &view(&cam), &all, Visibility { through: false, bvh: Some(&bvh) });
        assert_eq!(direct_sel, vec![0], "only the near plate is visible");
        let through = lasso_select(&mesh, &view(&cam), &all, Visibility { through: true, bvh: Some(&bvh) });
        assert_eq!(through, vec![0, 1]);
    }

    #[test]
    fn brush_stroke_paints_within_its_radius_and_covers_the_gaps() {
        let mesh = synth::sphere(Point3::origin(), 10.0, 48, 24);
        let cam = front_camera(&mesh);
        let (cx, cy) = (W / 2.0, H / 2.0);
        let one = brush_select(&mesh, &view(&cam), &[[cx, cy]], 25.0, direct());
        assert!(!one.is_empty());
        for &t in &one {
            let p = cam.project(&centroid(&mesh, t as usize).unwrap(), W, H).unwrap();
            assert!((p[0] - cx).hypot(p[1] - cy) <= 25.0 + 1e-9);
        }
        // a long stroke is sampled densely enough: every point on it is within the radius of a sample
        let pts = stroke_points([cx - 60.0, cy], [cx + 60.0, cy], 25.0);
        assert!(pts.len() >= 10 && pts[0] == [cx - 60.0, cy] && pts[pts.len() - 1] == [cx + 60.0, cy]);
        let stroke = brush_select(&mesh, &view(&cam), &pts, 25.0, direct());
        assert!(stroke.len() > 2 * one.len());
        // erasing the stroke from the selection leaves the rest
        let mut sel = TriSelection::default();
        sel.apply(SelectOp::Replace, &stroke);
        sel.apply(SelectOp::Subtract, &one);
        assert_eq!(sel.tris.len(), stroke.len() - one.iter().filter(|t| stroke.contains(t)).count());
        sel.apply(SelectOp::Add, &one);
        assert_eq!(sel.to_vec(), stroke.iter().copied().collect::<BTreeSet<u32>>().into_iter().collect::<Vec<_>>());
    }

    /// Smart select stops at the sharp edges of a box: it takes the one face under the seed.
    #[test]
    fn smart_select_grows_to_the_sharp_edge() {
        // an L of two quads: flat at the start, folding 90° at the middle
        let mut mesh = Mesh::default();
        let p = |x: f64, y: f64, z: f64| Point3::new(x, y, z);
        mesh.positions = vec![
            p(0.0, 0.0, 0.0),
            p(5.0, 0.0, 0.0),
            p(10.0, 0.0, 0.0),
            p(0.0, 5.0, 0.0),
            p(5.0, 5.0, 0.0),
            p(10.0, 5.0, 0.0),
            p(10.0, 0.0, 5.0),
            p(10.0, 5.0, 5.0),
        ];
        mesh.triangles = vec![[0, 1, 4], [0, 4, 3], [1, 2, 5], [1, 5, 4], [2, 6, 7], [2, 7, 5]];
        let flat = smart_select(&mesh, 0, 15.0);
        assert_eq!(flat, vec![0, 1, 2, 3], "the flat top, not the wall");
        let all = smart_select(&mesh, 0, 100.0);
        assert_eq!(all.len(), 6, "a 100° threshold crosses the fold");
        assert!(smart_select(&mesh, 0, 0.0).is_empty() && smart_select(&mesh, 0, f64::NAN).is_empty());
    }

    /// The ids are those of the full mesh, whatever level of detail is drawn: a selection made on
    /// the full mesh maps onto drawn triangles of the coarse mesh, and ids stay within the full mesh.
    #[test]
    fn selection_ids_are_original_and_map_onto_the_lod() {
        let full = synth::sphere(Point3::origin(), 10.0, 96, 48);
        let lod = surfacecraft_geom::lod::cluster(&full, 800).expect("valid mesh");
        assert!(lod.mesh.triangle_count() < full.triangle_count());
        let cam = front_camera(&full);
        let sel = brush_select(&full, &view(&cam), &[[W / 2.0, H / 2.0]], 30.0, direct());
        assert!(!sel.is_empty());
        assert!(sel.iter().all(|&t| (t as usize) < full.triangle_count()), "original ids");
        let drawn = lod.map_triangles(&sel);
        assert!(!drawn.is_empty() && drawn.len() < sel.len(), "several original triangles share a coarse one");
        assert!(drawn.iter().all(|&t| (t as usize) < lod.mesh.triangle_count()));
        // each coarse triangle is where the original ones were
        let cp = cam.project(&centroid(&lod.mesh, drawn[0] as usize).unwrap(), W, H).unwrap();
        assert!((cp[0] - W / 2.0).hypot(cp[1] - H / 2.0) < 80.0, "{cp:?}");
    }
}
