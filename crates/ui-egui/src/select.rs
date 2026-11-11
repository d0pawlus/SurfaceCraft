//! Triangle selection on meshes: brush, lasso and smart select, in screen space.
//!
//! Everything here is plain maths on a mesh and a camera (no GPU, no egui), so it is testable
//! with a synthetic mesh and a known camera. A selection always speaks of the triangle ids of the
//! *full-resolution* mesh: when the display uses a coarser level of detail (`geom::lod`), callers
//! pass the full mesh (`DisplayItem::pick_mesh`) and map the ids to drawn triangles only for
//! highlighting (`DisplayItem::drawn_triangles`).
//!
//! The screen tools search the mesh through its BVH: a box of the hierarchy whose projection misses
//! the brush or the lasso is skipped with all its triangles, so the cost follows the triangles near
//! the stroke, not the whole mesh. The result is the same as testing every triangle.
use crate::camera::Camera;
use std::collections::BTreeSet;
use surfacecraft_geom::{Aabb, Bvh, Mesh, Point3};

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

/// A mesh with the hierarchy the screen tools search it with (built once per mesh).
#[derive(Clone, Copy)]
pub struct Scan<'a> {
    /// The triangles, full-resolution ids.
    pub mesh: &'a Mesh,
    /// The hierarchy of `mesh`: the candidates of a stroke, and the hit test that tells hidden
    /// triangles from visible ones.
    pub bvh: &'a Bvh,
}

fn centroid(mesh: &Mesh, t: usize) -> Option<Point3> {
    let [a, b, c] = mesh.triangles.get(t)?;
    let (pa, pb, pc) =
        (mesh.positions.get(*a as usize)?, mesh.positions.get(*b as usize)?, mesh.positions.get(*c as usize)?);
    Some(Point3::from((pa.coords + pb.coords + pc.coords) / 3.0))
}

/// Whether the projection of the box `b` may reach the pixel box `bounds`. Conservative: a box
/// with a corner behind the eye, or not finite, is kept.
fn box_may_meet(camera: &Camera, w: f64, h: f64, b: &Aabb, bounds: [f64; 4]) -> bool {
    if b.is_empty() {
        return false;
    }
    let mut r = [f64::INFINITY, f64::INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY];
    for corner in 0..8u8 {
        let pick = |bit: u8, lo: f64, hi: f64| if corner & bit == 0 { lo } else { hi };
        let p = Point3::new(pick(1, b.min.x, b.max.x), pick(2, b.min.y, b.max.y), pick(4, b.min.z, b.max.z));
        let Some([x, y, _]) = camera.project(&p, w, h) else { return true };
        if !x.is_finite() || !y.is_finite() {
            return true;
        }
        r = [r[0].min(x), r[1].min(y), r[2].max(x), r[3].max(y)];
    }
    // a convex box projects inside the box of its projected corners, so this never drops a triangle
    r[0] <= bounds[2] && r[2] >= bounds[0] && r[1] <= bounds[3] && r[3] >= bounds[1]
}

/// Whether something other than triangle `t` lies between the eye and the centroid `c` of `t`.
fn occluded(bvh: &Bvh, eye: &Point3, c: &Point3, t: usize) -> bool {
    let to = c - eye;
    let dist = to.norm();
    let Some(dir) = surfacecraft_geom::normalize(to) else { return false };
    matches!(bvh.ray_cast(eye, &dir), Some((hit, hit_t)) if hit != t && hit_t < dist - dist * 1e-6 - 1e-9)
}

/// The per-triangle test of the screen tools: the centroid projects inside `bounds` and `inside`,
/// and unless `through` the triangle faces the eye and nothing covers it.
fn selected(
    scan: Scan<'_>,
    view: &View<'_>,
    bounds: [f64; 4],
    through: bool,
    inside: &dyn Fn(f64, f64) -> bool,
    t: usize,
) -> bool {
    let Some(c) = centroid(scan.mesh, t) else { return false };
    let Some([x, y, _]) = view.camera.project(&c, view.w, view.h) else { return false };
    if x < bounds[0] || x > bounds[2] || y < bounds[1] || y > bounds[3] || !inside(x, y) {
        return false;
    }
    if through {
        return true;
    }
    let eye = view.camera.eye();
    let Some(n) = scan.mesh.face_normal(t) else { return false };
    if n.dot(&(eye - c)) <= 0.0 {
        return false;
    }
    !occluded(scan.bvh, &eye, &c, t)
}

/// Triangles whose centroid projects to a pixel for which `inside` is true (checked inside the
/// pixel box `bounds = [min_x, min_y, max_x, max_y]` first), keeping only the triangles that face
/// the eye and that nothing covers unless `through`. Only the hierarchy boxes that the pixel box
/// reaches are tested.
#[must_use]
pub fn select_in_screen(
    scan: Scan<'_>,
    view: &View<'_>,
    bounds: [f64; 4],
    through: bool,
    inside: &dyn Fn(f64, f64) -> bool,
) -> Vec<u32> {
    let (w, h) = (view.w, view.h);
    scan.bvh
        .candidates(&|b| box_may_meet(view.camera, w, h, b, bounds))
        .into_iter()
        .filter(|&t| selected(scan, view, bounds, through, inside, t as usize))
        .collect()
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
pub fn lasso_select(scan: Scan<'_>, view: &View<'_>, poly: &[[f64; 2]], through: bool) -> Vec<u32> {
    if poly.len() < 3 || poly.iter().any(|p| !p[0].is_finite() || !p[1].is_finite()) {
        return Vec::new();
    }
    let mut b = [f64::INFINITY, f64::INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY];
    for p in poly {
        b = [b[0].min(p[0]), b[1].min(p[1]), b[2].max(p[0]), b[3].max(p[1])];
    }
    select_in_screen(scan, view, b, through, &|x, y| point_in_polygon(poly, x, y))
}

/// Brush: triangles within `radius` px of any of the `centres` (a stroke sampled by
/// [`stroke_points`]).
#[must_use]
pub fn brush_select(scan: Scan<'_>, view: &View<'_>, centres: &[[f64; 2]], radius: f64, through: bool) -> Vec<u32> {
    let radius = radius.clamp(1.0, 2000.0);
    if centres.is_empty() || centres.iter().any(|p| !p[0].is_finite() || !p[1].is_finite()) {
        return Vec::new();
    }
    let mut b = [f64::INFINITY, f64::INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY];
    for p in centres {
        b = [b[0].min(p[0] - radius), b[1].min(p[1] - radius), b[2].max(p[0] + radius), b[3].max(p[1] + radius)];
    }
    let r2 = radius * radius;
    select_in_screen(scan, view, b, through, &|x, y| {
        centres.iter().any(|c| (c[0] - x).powi(2) + (c[1] - y).powi(2) <= r2)
    })
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

    /// Every triangle goes through the per-triangle test: the reference the BVH search must agree
    /// with.
    fn brute(
        scan: Scan<'_>,
        view: &View<'_>,
        bounds: [f64; 4],
        through: bool,
        inside: &dyn Fn(f64, f64) -> bool,
    ) -> Vec<u32> {
        (0..scan.mesh.triangle_count())
            .filter(|&t| selected(scan, view, bounds, through, inside, t))
            .map(|t| t as u32)
            .collect()
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
        let bvh = Bvh::build(&mesh);
        let s = Scan { mesh: &mesh, bvh: &bvh };
        let cam = front_camera(&mesh);
        let all = [[0.0, 0.0], [W, 0.0], [W, H], [0.0, H]];
        let front = lasso_select(s, &view(&cam), &all, false);
        let through = lasso_select(s, &view(&cam), &all, true);
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
        let bvh = Bvh::build(&mesh);
        let s = Scan { mesh: &mesh, bvh: &bvh };
        let cam = front_camera(&mesh);
        let (cx, cy) = (W / 2.0, H / 2.0);
        let poly = [[cx - 30.0, cy - 30.0], [cx + 30.0, cy - 30.0], [cx + 30.0, cy + 30.0], [cx - 30.0, cy + 30.0]];
        let sel = lasso_select(s, &view(&cam), &poly, false);
        assert!(!sel.is_empty());
        for &t in &sel {
            let p = cam.project(&centroid(&mesh, t as usize).unwrap(), W, H).unwrap();
            assert!((p[0] - cx).abs() <= 30.0 && (p[1] - cy).abs() <= 30.0);
        }
        let big = [[0.0, 0.0], [W, 0.0], [W, H], [0.0, H]];
        assert!(sel.len() < lasso_select(s, &view(&cam), &big, false).len() / 2);
        // a degenerate polygon selects nothing
        assert!(lasso_select(s, &view(&cam), &poly[..2], false).is_empty());
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
        let s = Scan { mesh: &mesh, bvh: &bvh };
        let direct_sel = lasso_select(s, &view(&cam), &all, false);
        assert_eq!(direct_sel, vec![0], "only the near plate is visible");
        let through = lasso_select(s, &view(&cam), &all, true);
        assert_eq!(through, vec![0, 1]);
    }

    #[test]
    fn brush_stroke_paints_within_its_radius_and_covers_the_gaps() {
        let mesh = synth::sphere(Point3::origin(), 10.0, 48, 24);
        let bvh = Bvh::build(&mesh);
        let s = Scan { mesh: &mesh, bvh: &bvh };
        let cam = front_camera(&mesh);
        let (cx, cy) = (W / 2.0, H / 2.0);
        let one = brush_select(s, &view(&cam), &[[cx, cy]], 25.0, false);
        assert!(!one.is_empty());
        for &t in &one {
            let p = cam.project(&centroid(&mesh, t as usize).unwrap(), W, H).unwrap();
            assert!((p[0] - cx).hypot(p[1] - cy) <= 25.0 + 1e-9);
        }
        // a long stroke is sampled densely enough: every point on it is within the radius of a sample
        let pts = stroke_points([cx - 60.0, cy], [cx + 60.0, cy], 25.0);
        assert!(pts.len() >= 10 && pts[0] == [cx - 60.0, cy] && pts[pts.len() - 1] == [cx + 60.0, cy]);
        let stroke = brush_select(s, &view(&cam), &pts, 25.0, false);
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
        let bvh = Bvh::build(&full);
        let sel = brush_select(Scan { mesh: &full, bvh: &bvh }, &view(&cam), &[[W / 2.0, H / 2.0]], 30.0, false);
        assert!(!sel.is_empty());
        assert!(sel.iter().all(|&t| (t as usize) < full.triangle_count()), "original ids");
        let drawn = lod.map_triangles(&sel);
        assert!(!drawn.is_empty() && drawn.len() < sel.len(), "several original triangles share a coarse one");
        assert!(drawn.iter().all(|&t| (t as usize) < lod.mesh.triangle_count()));
        // each coarse triangle is where the original ones were
        let cp = cam.project(&centroid(&lod.mesh, drawn[0] as usize).unwrap(), W, H).unwrap();
        assert!((cp[0] - W / 2.0).hypot(cp[1] - H / 2.0) < 80.0, "{cp:?}");
    }

    /// The pixel box of a brush over `centres` (the box `brush_select` tests first).
    fn brush_bounds(centres: &[[f64; 2]], r: f64) -> [f64; 4] {
        let mut b = [f64::INFINITY, f64::INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY];
        for p in centres {
            b = [b[0].min(p[0] - r), b[1].min(p[1] - r), b[2].max(p[0] + r), b[3].max(p[1] + r)];
        }
        b
    }

    /// Two plates one behind the other, both facing the eye: the rear one is covered.
    fn two_plates() -> Mesh {
        let mut mesh = Mesh::default();
        for y in [0.0, 5.0] {
            let b = mesh.positions.len() as u32;
            mesh.positions.push(Point3::new(0.0, y, 0.0));
            mesh.positions.push(Point3::new(10.0, y, 0.0));
            mesh.positions.push(Point3::new(0.0, y, 10.0));
            mesh.triangles.push([b, b + 1, b + 2]);
        }
        mesh
    }

    /// The BVH search returns exactly what testing every triangle returns, for every tool, view,
    /// visibility and stroke used here (a sphere, a plate pair with hidden triangles, a sphere seen
    /// from above).
    #[test]
    fn bvh_search_selects_what_testing_every_triangle_selects() {
        let meshes = [synth::sphere(Point3::origin(), 10.0, 48, 24), two_plates()];
        let mut rng = surfacecraft_geom::Rng::new(7);
        for mesh in &meshes {
            let bvh = Bvh::build(mesh);
            let s = Scan { mesh, bvh: &bvh };
            for standard in [crate::camera::StandardView::Front, crate::camera::StandardView::Top] {
                let mut cam = Camera::default();
                cam.fit(&mesh.bbox());
                cam.set_standard(standard);
                let v = view(&cam);
                for through in [false, true] {
                    let all = [[0.0, 0.0], [W, 0.0], [W, H], [0.0, H]];
                    let poly = [[W * 0.2, H * 0.1], [W * 0.9, H * 0.3], [W * 0.5, H * 0.95]];
                    let inside_all = |x: f64, y: f64| point_in_polygon(&all, x, y);
                    let bounds = [0.0, 0.0, W, H];
                    assert_eq!(
                        select_in_screen(s, &v, bounds, through, &inside_all),
                        brute(s, &v, bounds, through, &inside_all)
                    );
                    let inside_tri = |x: f64, y: f64| point_in_polygon(&poly, x, y);
                    let tri_bounds = [W * 0.2, H * 0.1, W * 0.9, H * 0.95];
                    assert_eq!(
                        select_in_screen(s, &v, tri_bounds, through, &inside_tri),
                        brute(s, &v, tri_bounds, through, &inside_tri)
                    );
                    assert_eq!(lasso_select(s, &v, &poly, through), brute(s, &v, tri_bounds, through, &inside_tri));
                    for _ in 0..20 {
                        let c = [rng.range(0.0, W), rng.range(0.0, H)];
                        let r = rng.range(3.0, 60.0);
                        let stroke = [c, [c[0] + rng.range(-40.0, 40.0), c[1] + rng.range(-40.0, 40.0)]];
                        let centres = stroke_points(stroke[0], stroke[1], r);
                        let inside =
                            |x: f64, y: f64| centres.iter().any(|q| (q[0] - x).powi(2) + (q[1] - y).powi(2) <= r * r);
                        let got = brush_select(s, &v, &centres, r, through);
                        let want = brute(s, &v, brush_bounds(&centres, r), through, &inside);
                        assert_eq!(got, want, "stroke at {c:?} radius {r}, through {through}");
                    }
                }
            }
        }
    }

    /// Timing on about a million triangles: the brush search against testing every triangle. Run
    /// with `cargo test -p surfacecraft-ui-egui --profile ci brush_on_a_million -- --ignored --nocapture`.
    #[test]
    #[ignore = "timing on about 1 M triangles"]
    fn brush_on_a_million_triangles_beats_testing_every_triangle() {
        let mesh = synth::sphere(Point3::origin(), 10.0, 720, 720);
        let bvh = Bvh::build(&mesh);
        let s = Scan { mesh: &mesh, bvh: &bvh };
        let mut cam = Camera::default();
        cam.fit(&mesh.bbox());
        cam.set_standard(crate::camera::StandardView::Front);
        let v = view(&cam);
        let centres = stroke_points([W / 2.0 - 30.0, H / 2.0], [W / 2.0 + 30.0, H / 2.0], 24.0);
        let inside = |x: f64, y: f64| centres.iter().any(|q| (q[0] - x).powi(2) + (q[1] - y).powi(2) <= 24.0 * 24.0);
        let bounds = [0.0, 0.0, W, H];
        let t0 = std::time::Instant::now();
        let got = brush_select(s, &v, &centres, 24.0, false);
        let search = t0.elapsed();
        let t1 = std::time::Instant::now();
        let want = brute(s, &v, bounds, false, &inside);
        let every = t1.elapsed();
        eprintln!(
            "{} triangles: BVH search {:.2} ms, every triangle {:.2} ms, {} selected",
            mesh.triangle_count(),
            search.as_secs_f64() * 1e3,
            every.as_secs_f64() * 1e3,
            got.len()
        );
        assert_eq!(got, want);
        assert!(search < every, "the search must beat testing every triangle");
    }
}
