//! The hole and the moves that keep its patch sound: pinned corners, aligned edges and the thinnest triangles.

use super::sides::*;
use super::*;

/// Area of the smallest patch triangle (from `first_tri` on) at vertex `v`.
fn thinnest_at(mesh: &Mesh, first_tri: usize, v: u32) -> f64 {
    (first_tri..mesh.triangles.len())
        .filter(|&t| mesh.triangles.get(t).is_some_and(|c| c.contains(&v)))
        .map(|t| mesh.face_area(t))
        .fold(f64::INFINITY, f64::min)
}

/// Moves vertex `v` to `to` unless that leaves a patch triangle thinner than `min_area`.
pub(super) fn move_keeping_shape(mesh: &mut Mesh, first_tri: usize, v: usize, to: Point3, min_area: f64) {
    let Some(from) = mesh.positions.get(v).copied() else { return };
    if let Some(p) = mesh.positions.get_mut(v) {
        *p = to;
    }
    if thinnest_at(mesh, first_tri, v as u32) < min_area
        && let Some(p) = mesh.positions.get_mut(v)
    {
        *p = from;
    }
}

/// The hole: its patch and the measures of its border.
pub(super) struct Hole {
    pub(super) first_tri: usize,
    pub(super) first_vertex: usize,
    /// Middle of the border.
    pub(super) middle: Point3,
    /// Largest distance of a border point from the middle.
    pub(super) radius: f64,
    /// Mean length of a border edge.
    pub(super) edge: f64,
}

impl Hole {
    pub(super) fn new(mesh: &Mesh, first_tri: usize, first_vertex: usize, lp: &[u32]) -> Option<Hole> {
        let on: Vec<Point3> = lp.iter().filter_map(|&v| mesh.positions.get(v as usize).copied()).collect();
        if on.len() < 3 || on.len() != lp.len() || first_vertex >= mesh.positions.len() {
            return None;
        }
        let n = on.len();
        let middle = Point3::from(on.iter().fold(Vec3::zeros(), |a, p| a + p.coords) / n as f64);
        let radius = on.iter().map(|p| (p - middle).norm()).fold(0.0, f64::max);
        let edge = (0..n).filter_map(|k| Some((on.get(k)? - on.get((k + 1) % n)?).norm())).sum::<f64>() / n as f64;
        (edge > 0.0 && radius.is_finite()).then_some(Hole { first_tri, first_vertex, middle, radius, edge })
    }

    pub(super) fn min_area(&self) -> f64 {
        MIN_AREA * self.edge * self.edge
    }
}

/// Where three sides meet in a point inside the hole there is a corner, and a patch has to have
/// a vertex on it or a triangle cuts it off: the patch vertex nearest the corner is moved there.
pub(super) fn pin_corner(mesh: &mut Mesh, hole: &Hole, wedge: &Wedge) {
    if wedge.sides.len() != MAX_SIDES {
        return;
    }
    let all: Vec<&Side> = wedge.sides.iter().collect();
    let corner = onto_all(&all, hole.middle);
    // three sides need not meet in a point (a chamfer between a face and a wall), and a corner
    // outside the hole is not the patch's to make
    let off = wedge.sides.iter().map(|s| s.height(&corner).abs()).fold(0.0, f64::max);
    if !crate::finite(&corner)
        || (corner - hole.middle).norm() > hole.radius
        || !(off <= CORNER_TOLERANCE * hole.radius)
    {
        return;
    }
    let nearest = mesh
        .positions
        .iter()
        .enumerate()
        .skip(hole.first_vertex)
        .map(|(i, p)| ((p - corner).norm(), i))
        .fold(None, |b: Option<(f64, usize)>, c| if b.is_none_or(|b| c.0 < b.0) { Some(c) } else { b });
    if let Some((_, i)) = nearest {
        move_keeping_shape(mesh, hole.first_tri, i, corner, hole.min_area());
    }
}

/// The two patch triangles on edge (a, b) and the vertices opposite it, when it is an inner
/// patch edge that may be flipped.
type Flip = (usize, usize, u32, u32);

fn flip_of(mesh: &Mesh, edge: &HashMap<(u32, u32), (usize, usize)>, a: u32, b: u32) -> Option<Flip> {
    let (&(t0, k0), &(t1, k1)) = (edge.get(&(a, b))?, edge.get(&(b, a))?);
    let c = mesh.triangles.get(t0)?[(k0 + 2) % 3];
    let d = mesh.triangles.get(t1)?[(k1 + 2) % 3];
    (c != d && !edge.contains_key(&(c, d)) && !edge.contains_key(&(d, c))).then_some((t0, t1, c, d))
}

/// Should edge (a, b) between triangles (a, b, c) and (b, a, d) become (c, d)? Yes when the
/// middle of (c, d) is at most [`FLIP_GAIN`] × as far from the wedge and the two new triangles
/// keep the orientation and are not slivers.
fn flip_gains(mesh: &Mesh, wedge: &Wedge, min_area: f64, [a, b, c, d]: [u32; 4]) -> bool {
    let at = |v: u32| mesh.positions.get(v as usize).copied();
    let (Some(pa), Some(pb), Some(pc), Some(pd)) = (at(a), at(b), at(c), at(d)) else { return false };
    let mid = |p: &Point3, q: &Point3| Point3::from((p.coords + q.coords) / 2.0);
    if !(wedge.off(&mid(&pc, &pd)) < FLIP_GAIN * wedge.off(&mid(&pa, &pb))) {
        return false;
    }
    // the new triangles (c, d, b) and (d, c, a) must face the way the old pair did
    let old = tri::cross(&pa, &pb, &pc) + tri::cross(&pb, &pa, &pd);
    let (n1, n2) = (tri::cross(&pc, &pd, &pb), tri::cross(&pd, &pc, &pa));
    n1.dot(&old) > 0.0 && n2.dot(&old) > 0.0 && n1.norm().min(n2.norm()) >= 2.0 * min_area
}

/// Flips patch edges that cut across a sharp edge of the wedge (see [`flip_gains`]).
pub(super) fn align_edges(mesh: &mut Mesh, hole: &Hole, wedge: &Wedge) {
    // a flip must not put a patch edge on an edge of the mesh around the patch
    let outside = crate::holes::outside_edges(mesh, hole.first_tri);
    for _pass in 0..FLIP_PASSES {
        let mut edge: HashMap<(u32, u32), (usize, usize)> = HashMap::new();
        for (ti, t) in mesh.triangles.iter().enumerate().skip(hole.first_tri) {
            for k in 0..3 {
                edge.insert((t[k], t[(k + 1) % 3]), (ti, k));
            }
        }
        let mut keys: Vec<(u32, u32)> = edge.keys().copied().filter(|(a, b)| a < b).collect();
        keys.sort_unstable();
        let mut done: HashSet<usize> = HashSet::new();
        let mut made: HashSet<(u32, u32)> = HashSet::new();
        for (a, b) in keys {
            let Some((t0, t1, c, d)) = flip_of(mesh, &edge, a, b) else { continue };
            let fresh = !done.contains(&t0)
                && !done.contains(&t1)
                && !made.contains(&(c.min(d), c.max(d)))
                && !outside.contains(&(c.min(d), c.max(d)));
            if !fresh || !flip_gains(mesh, wedge, hole.min_area(), [a, b, c, d]) {
                continue;
            }
            if let Some(t) = mesh.triangles.get_mut(t0) {
                *t = [c, d, b];
            }
            if let Some(t) = mesh.triangles.get_mut(t1) {
                *t = [d, c, a];
            }
            done.extend([t0, t1]);
            made.insert((c.min(d), c.max(d)));
        }
        if done.is_empty() {
            break;
        }
    }
}

/// Area of the thinnest patch triangle.
pub(super) fn thinnest(mesh: &Mesh, first_tri: usize) -> f64 {
    (first_tri..mesh.triangles.len()).map(|t| mesh.face_area(t)).fold(f64::INFINITY, f64::min)
}

/// Does patch triangle `t` face the way a side at its middle does? At an edge the middle is on
/// two sides at once (within the snap distance), and either will do.
pub(super) fn faces_out(mesh: &Mesh, hole: &Hole, wedge: &Wedge, t: usize) -> bool {
    let snap = EDGE_SNAP * hole.edge;
    let (Some(c), Some(n)) = (mesh.face_centroid(t), mesh.face_normal(t)) else { return false };
    let Some((_, top)) = wedge.top(&c) else { return false };
    wedge.sides.iter().any(|s| wedge.sign() * s.height(&c) >= top - snap && s.n.dot(&n) > MIN_FACING)
}
