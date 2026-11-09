//! Picking the faces and edges of B-rep bodies in the viewport.
//!
//! A body is drawn as its display tessellation; `BodyTopo` (from the engine) says which B-rep face
//! every triangle belongs to and gives the edge polylines, both indexed as in the body record, so
//! the picked indices go straight into the `faces` / `edges` parameters of feature commands.
use crate::select::View;
use std::collections::BTreeSet;
use surfacecraft_doc::ObjectId;
use surfacecraft_engine::display::BodyTopo;
use surfacecraft_geom::{Bvh, Point3, Vec3};

/// What a click can pick on a body.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BrepTarget {
    /// Faces.
    Faces,
    /// Edges.
    Edges,
}

/// One picked element.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BrepHit {
    /// A face, by index in the body record.
    Face(usize),
    /// An edge, by index in the body record.
    Edge(usize),
}

/// The faces and edges selected on one body.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct BrepSelection {
    /// The body they belong to.
    pub body: Option<ObjectId>,
    /// Selected faces.
    pub faces: BTreeSet<usize>,
    /// Selected edges.
    pub edges: BTreeSet<usize>,
}

impl BrepSelection {
    /// Whether nothing is selected.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.faces.is_empty() && self.edges.is_empty()
    }

    /// Applies a click on `hit` of `body`: without `add` the hit replaces the selection (a click
    /// on nothing clears it); with `add` it is toggled.
    pub fn click(&mut self, body: ObjectId, hit: Option<BrepHit>, add: bool) {
        if self.body != Some(body) {
            self.faces.clear();
            self.edges.clear();
            self.body = Some(body);
        }
        if !add {
            let keep = hit.is_some_and(|h| match h {
                BrepHit::Face(f) => self.faces.len() == 1 && self.edges.is_empty() && self.faces.contains(&f),
                BrepHit::Edge(e) => self.edges.len() == 1 && self.faces.is_empty() && self.edges.contains(&e),
            });
            self.faces.clear();
            self.edges.clear();
            if keep {
                // a plain click on the only selected element deselects it
                return;
            }
        }
        match hit {
            Some(BrepHit::Face(f)) => {
                if !self.faces.remove(&f) {
                    self.faces.insert(f);
                }
            }
            Some(BrepHit::Edge(e)) => {
                if !self.edges.remove(&e) {
                    self.edges.insert(e);
                }
            }
            None => {}
        }
    }

    /// Selected faces as the `faces` parameter of a command.
    #[must_use]
    pub fn face_list(&self) -> Vec<usize> {
        self.faces.iter().copied().collect()
    }

    /// Selected edges as the `edges` parameter of a command.
    #[must_use]
    pub fn edge_list(&self) -> Vec<usize> {
        self.edges.iter().copied().collect()
    }
}

/// The face under the ray: `(face, distance along the ray)`.
#[must_use]
pub fn pick_face(topo: &BodyTopo, bvh: &Bvh, origin: &Point3, dir: &Vec3) -> Option<(usize, f64)> {
    let (tri, t) = bvh.ray_cast(origin, dir)?;
    topo.face_of(tri as u32).map(|f| (f as usize, t))
}

fn dist_to_segment(p: [f64; 2], a: [f64; 2], b: [f64; 2]) -> (f64, f64) {
    let (dx, dy) = (b[0] - a[0], b[1] - a[1]);
    let l2 = dx * dx + dy * dy;
    let k = if l2 < 1e-12 { 0.0 } else { (((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / l2).clamp(0.0, 1.0) };
    let (qx, qy) = (a[0] + dx * k, a[1] + dy * k);
    ((p[0] - qx).hypot(p[1] - qy), k)
}

/// The edge nearest the pixel `cursor` within `tol_px`: `(edge, distance px)`. `surface_depth` is
/// the depth (along the view direction) of the body's surface under the cursor; edges behind it by
/// more than a few pixels are hidden and not picked. `None` for it means no occlusion test.
#[must_use]
pub fn pick_edge(
    topo: &BodyTopo,
    view: &View<'_>,
    cursor: [f64; 2],
    tol_px: f64,
    surface_depth: Option<f64>,
) -> Option<(usize, f64)> {
    let mut best: Option<(usize, f64)> = None;
    for (ei, poly) in topo.edges.iter().enumerate() {
        for seg in poly.windows(2) {
            let (Some(a), Some(b)) = (seg.first(), seg.get(1)) else { continue };
            let (Some(pa), Some(pb)) = (view.camera.project(a, view.w, view.h), view.camera.project(b, view.w, view.h))
            else {
                continue;
            };
            let (d, k) = dist_to_segment(cursor, [pa[0], pa[1]], [pb[0], pb[1]]);
            if d > tol_px || best.is_some_and(|(_, bd)| d >= bd) {
                continue;
            }
            if let Some(sd) = surface_depth {
                let depth = pa[2] + (pb[2] - pa[2]) * k;
                let slack = view.camera.world_per_pixel(depth, view.h) * (tol_px + 2.0);
                if depth > sd + slack {
                    continue;
                }
            }
            best = Some((ei, d));
        }
    }
    best
}

/// What a click at pixel `cursor` picks on a body, with its score (distance along the ray for a
/// face, distance in pixels for an edge; smaller is nearer). With edges as the target only edges
/// are looked at (the face under the cursor only hides the ones behind it).
#[must_use]
pub fn pick(
    topo: &BodyTopo,
    bvh: &Bvh,
    view: &View<'_>,
    cursor: [f64; 2],
    target: BrepTarget,
    tol_px: f64,
) -> Option<(BrepHit, f64)> {
    let (origin, dir) = view.camera.ray(cursor[0], cursor[1], view.w, view.h);
    let face = pick_face(topo, bvh, &origin, &dir);
    match target {
        BrepTarget::Faces => face.map(|(f, t)| (BrepHit::Face(f), t)),
        BrepTarget::Edges => {
            let (_, _, fwd) = view.camera.basis();
            let surface_depth = face.map(|(_, t)| t * dir.dot(&fwd));
            pick_edge(topo, view, cursor, tol_px, surface_depth).map(|(e, d)| (BrepHit::Edge(e), d))
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::camera::{Camera, StandardView};
    use surfacecraft_geom::Mesh;

    /// A unit-ish cube as a body display: 6 faces of 2 triangles, 12 edges.
    fn cube() -> (Mesh, BodyTopo) {
        let mut mesh = Mesh::default();
        let mut tri_face = Vec::new();
        // faces in order: -Y (front), +Y, -X, +X, -Z, +Z
        let corners = |x: f64, y: f64, z: f64| Point3::new(x * 10.0, y * 10.0, z * 10.0);
        let quads: [[(f64, f64, f64); 4]; 6] = [
            [(0., 0., 0.), (1., 0., 0.), (1., 0., 1.), (0., 0., 1.)],
            [(0., 1., 0.), (0., 1., 1.), (1., 1., 1.), (1., 1., 0.)],
            [(0., 0., 0.), (0., 0., 1.), (0., 1., 1.), (0., 1., 0.)],
            [(1., 0., 0.), (1., 1., 0.), (1., 1., 1.), (1., 0., 1.)],
            [(0., 0., 0.), (0., 1., 0.), (1., 1., 0.), (1., 0., 0.)],
            [(0., 0., 1.), (1., 0., 1.), (1., 1., 1.), (0., 1., 1.)],
        ];
        for (f, q) in quads.iter().enumerate() {
            let b = mesh.positions.len() as u32;
            for c in q {
                mesh.positions.push(corners(c.0, c.1, c.2));
            }
            mesh.triangles.push([b, b + 1, b + 2]);
            mesh.triangles.push([b, b + 2, b + 3]);
            tri_face.extend([f as u32; 2]);
        }
        let v = |x: f64, y: f64, z: f64| corners(x, y, z);
        let mut edges = Vec::new();
        for (a, b) in [
            ((0., 0., 0.), (1., 0., 0.)),
            ((1., 0., 0.), (1., 0., 1.)),
            ((1., 0., 1.), (0., 0., 1.)),
            ((0., 0., 1.), (0., 0., 0.)),
            ((0., 1., 0.), (1., 1., 0.)),
            ((1., 1., 0.), (1., 1., 1.)),
            ((1., 1., 1.), (0., 1., 1.)),
            ((0., 1., 1.), (0., 1., 0.)),
            ((0., 0., 0.), (0., 1., 0.)),
            ((1., 0., 0.), (1., 1., 0.)),
            ((1., 0., 1.), (1., 1., 1.)),
            ((0., 0., 1.), (0., 1., 1.)),
        ] {
            edges.push(vec![v(a.0, a.1, a.2), v(b.0, b.1, b.2)]);
        }
        (mesh, BodyTopo::new(tri_face, edges))
    }

    const W: f64 = 400.0;
    const H: f64 = 300.0;

    fn setup(view: StandardView) -> (Camera, Mesh, BodyTopo, Bvh) {
        let (mesh, topo) = cube();
        let mut cam = Camera::default();
        cam.fit(&mesh.bbox());
        cam.set_standard(view);
        let bvh = Bvh::build(&mesh);
        (cam, mesh, topo, bvh)
    }

    #[test]
    fn face_ranges_group_the_triangles_of_each_face() {
        let (_, topo) = cube();
        assert_eq!(topo.face_ranges.len(), 6);
        assert_eq!(topo.face_ranges[2], (2, 4, 6));
        assert_eq!(topo.triangles_of_faces(&[1, 5]), vec![2, 3, 10, 11]);
        assert_eq!(topo.face_of(7), Some(3));
        assert_eq!(topo.face_of(99), None);
    }

    /// From the front the centre click hits the -Y face (index 0); from above, the +Z face (5).
    #[test]
    fn clicking_the_centre_picks_the_face_that_looks_at_the_camera() {
        for (view, face) in
            [(StandardView::Front, 0), (StandardView::Back, 1), (StandardView::Top, 5), (StandardView::Right, 3)]
        {
            let (cam, _mesh, topo, bvh) = setup(view);
            let v = View { camera: &cam, w: W, h: H };
            let hit = pick(&topo, &bvh, &v, [W / 2.0, H / 2.0], BrepTarget::Faces, 6.0).map(|h| h.0);
            assert_eq!(hit, Some(BrepHit::Face(face)), "{view:?}");
        }
    }

    /// A click near the top edge of the front face picks that edge (the one at z = 10, y = 0 is
    /// edge 2); far from every edge there is none; edges at the back are hidden by the cube.
    #[test]
    fn edges_are_picked_within_the_tolerance_and_hidden_ones_are_not() {
        let (cam, _mesh, topo, bvh) = setup(StandardView::Front);
        let v = View { camera: &cam, w: W, h: H };
        let mid_top = cam.project(&Point3::new(5.0, 0.0, 10.0), W, H).unwrap();
        let hit = pick(&topo, &bvh, &v, [mid_top[0], mid_top[1] + 2.0], BrepTarget::Edges, 6.0).map(|h| h.0);
        assert_eq!(hit, Some(BrepHit::Edge(2)));
        let centre = pick(&topo, &bvh, &v, [W / 2.0, H / 2.0], BrepTarget::Edges, 6.0).map(|h| h.0);
        assert_eq!(centre, None, "the middle of a face is not near an edge");
        // the back-bottom edge (4) is behind the cube: found without an occlusion test, hidden by
        // the face the ray reaches first
        let back = cam.project(&Point3::new(5.0, 10.0, 0.0), W, H).unwrap();
        let cur = [back[0], back[1]];
        assert_eq!(pick_edge(&topo, &v, cur, 4.0, None).map(|e| e.0), Some(4));
        assert_eq!(pick(&topo, &bvh, &v, cur, BrepTarget::Edges, 4.0).map(|h| h.0), None, "hidden by the front face");
    }

    #[test]
    fn selection_clicks_toggle_replace_and_clear() {
        let body = ObjectId(3);
        let mut s = BrepSelection::default();
        s.click(body, Some(BrepHit::Face(2)), false);
        s.click(body, Some(BrepHit::Face(4)), true);
        assert_eq!(s.face_list(), vec![2, 4]);
        s.click(body, Some(BrepHit::Face(2)), true);
        assert_eq!(s.face_list(), vec![4], "shift toggles off");
        s.click(body, Some(BrepHit::Edge(7)), false);
        assert_eq!((s.face_list(), s.edge_list()), (vec![], vec![7]), "a plain click replaces");
        s.click(body, Some(BrepHit::Edge(7)), false);
        assert!(s.is_empty(), "a plain click on the only selected element deselects it");
        s.click(body, Some(BrepHit::Edge(1)), false);
        s.click(body, None, false);
        assert!(s.is_empty(), "a click on nothing clears");
        s.click(body, Some(BrepHit::Face(1)), false);
        s.click(ObjectId(9), Some(BrepHit::Face(0)), true);
        assert_eq!((s.body, s.face_list()), (Some(ObjectId(9)), vec![0]), "another body starts afresh");
    }
}
