//! Finding the folds: triangles that face against the surface around them, grouped where they touch.
use crate::{Aabb, Bvh, Mesh, Point3, Vec3, par, tri};
use std::collections::HashMap;

/// A vertex is read for at most this many incident triangles (a hub of a fan).
pub(super) const MAX_ROW: usize = 64;
/// Triangles read as candidates for a crossing of one triangle (a pile of coincident sheets stops there).
const MAX_CANDIDATES: usize = 256;
/// Two triangles on an edge are folded flat onto each other when their normals are this close to opposite
/// and their third corners lie on the same side of the edge.
const FOLDED_FLAT_COS: f64 = 0.9;
/// Edges sampled for the median edge length.
const EDGE_SAMPLE_TRIANGLES: usize = 50_000;
/// A cell index stays within this magnitude (the coordinates over the cell side): far inside `i64`, so
/// that the neighbours of a cell can be addressed without overflow. A mesh beyond it is left alone.
const MAX_CELL_INDEX: f64 = 1.0e12;

/// Triangles around each vertex (compressed rows).
pub(super) struct Incidence {
    start: Vec<usize>,
    tris: Vec<u32>,
}

impl Incidence {
    pub(super) fn new(mesh: &Mesh) -> Self {
        let mut start = vec![0usize; mesh.positions.len() + 1];
        for t in &mesh.triangles {
            for &v in t {
                if let Some(s) = start.get_mut(v as usize + 1) {
                    *s += 1;
                }
            }
        }
        for i in 1..start.len() {
            start[i] += start[i - 1];
        }
        let mut fill = start.clone();
        let mut tris = vec![0u32; mesh.triangles.len() * 3];
        for (i, t) in mesh.triangles.iter().enumerate() {
            for &v in t {
                if let Some(f) = fill.get_mut(v as usize)
                    && let Some(slot) = tris.get_mut(*f)
                {
                    *slot = i as u32;
                    *f += 1;
                }
            }
        }
        Self { start, tris }
    }

    pub(super) fn row(&self, v: u32) -> &[u32] {
        let (a, b) =
            (self.start.get(v as usize).copied().unwrap_or(0), self.start.get(v as usize + 1).copied().unwrap_or(0));
        self.tris.get(a..b).unwrap_or(&[])
    }
}

/// Unit normal and area of every triangle (a degenerate one has a zero normal and area 0).
pub(super) fn facets(mesh: &Mesh) -> Vec<(Vec3, f64)> {
    par::map(mesh.triangles.len(), |i| {
        let area = mesh.face_area(i);
        match mesh.face_normal(i) {
            Some(n) if area.is_finite() && area > 0.0 => (n, area),
            _ => (Vec3::zeros(), 0.0),
        }
    })
}

type Cell = (i64, i64, i64);

/// The cell of `p`; `None` when its index would leave [`MAX_CELL_INDEX`] (or is not a number).
fn cell_of(p: &Point3, inv: f64) -> Option<Cell> {
    let index = |x: f64| Some((x * inv).floor()).filter(|c| c.abs() <= MAX_CELL_INDEX).map(|c| c as i64);
    Some((index(p.x)?, index(p.y)?, index(p.z)?))
}

/// Which triangles face against their surroundings: the angle to the area-weighted normal of the
/// triangles in the 3 x 3 x 3 cells of side `cell` around their centroid is more than `flip_cos` past
/// a right angle, and those triangles face one way (`coherence`: the length of the summed area-weighted
/// normals over the summed areas), so that the two faces of a thin plate, or a sharp edge, are not told
/// apart by chance. The cell is several edges across: a fold is a minority among them, which a ring
/// or two of triangles would not make it.
pub(super) fn flipped(mesh: &Mesh, facets: &[(Vec3, f64)], cell: f64, flip_cos: f64, coherence: f64) -> Vec<bool> {
    let inv = if cell.is_finite() && cell > 0.0 { 1.0 / cell } else { return vec![false; mesh.triangles.len()] };
    let centroids: Vec<Option<Cell>> =
        (0..mesh.triangles.len()).map(|t| mesh.face_centroid(t).and_then(|c| cell_of(&c, inv))).collect();
    if centroids.iter().any(Option::is_none) {
        // a coordinate far beyond the cell side: nothing is told apart rather than risk the arithmetic
        return vec![false; mesh.triangles.len()];
    }
    let mut sums: HashMap<Cell, (Vec3, f64)> = HashMap::new();
    for (c, f) in centroids.iter().zip(facets) {
        if let Some(c) = c {
            let e = sums.entry(*c).or_insert((Vec3::zeros(), 0.0));
            e.0 += f.0 * f.1;
            e.1 += f.1;
        }
    }
    par::map(mesh.triangles.len(), |t| {
        let (Some(&(n, area)), Some(Some(c))) = (facets.get(t), centroids.get(t)) else { return false };
        if area <= 0.0 {
            return false;
        }
        let (mut sum, mut total) = (Vec3::zeros(), 0.0);
        for (dx, dy, dz) in (-1..=1).flat_map(|x| (-1..=1).flat_map(move |y| (-1..=1).map(move |z| (x, y, z)))) {
            if let Some(e) = sums.get(&(c.0 + dx, c.1 + dy, c.2 + dz)) {
                sum += e.0;
                total += e.1;
            }
        }
        let len = sum.norm();
        total > 0.0 && len >= coherence * total && n.dot(&sum) < -flip_cos * len
    })
}

/// A group of flipped triangles that touch each other, and what it measures.
#[derive(Clone, Debug)]
pub(super) struct Site {
    pub triangles: Vec<u32>,
    pub area: f64,
    pub extent: f64,
    /// Largest distance of a vertex of the group from the plane of the unflipped triangles around it
    /// (infinite when there are none).
    pub height: f64,
    /// A triangle of the group has a boundary edge.
    pub on_rim: bool,
    /// A triangle of the group passes through a triangle it shares no vertex with.
    pub crossing: bool,
}

fn find(p: &mut [u32], mut x: u32) -> u32 {
    while let Some(&up) = p.get(x as usize) {
        if up == x {
            break;
        }
        let grand = p.get(up as usize).copied().unwrap_or(up);
        if let Some(s) = p.get_mut(x as usize) {
            *s = grand;
        }
        x = grand;
    }
    x
}

/// The flipped triangles grouped by shared vertices, in ascending order of their first triangle.
fn groups(mesh: &Mesh, mask: &[bool]) -> Vec<Vec<u32>> {
    let mut parent: Vec<u32> = (0..mesh.positions.len() as u32).collect();
    for t in mesh.triangles.iter().enumerate().filter(|(i, _)| mask.get(*i).copied().unwrap_or(false)).map(|(_, t)| t) {
        for &v in t.iter().skip(1) {
            let (a, b) = (find(&mut parent, t[0]), find(&mut parent, v));
            if a != b
                && let Some(s) = parent.get_mut(b as usize)
            {
                *s = a;
            }
        }
    }
    let mut by_root: std::collections::BTreeMap<u32, Vec<u32>> = Default::default();
    for (i, t) in mesh.triangles.iter().enumerate() {
        if mask.get(i).copied().unwrap_or(false) {
            by_root.entry(find(&mut parent, t[0])).or_default().push(i as u32);
        }
    }
    let mut out: Vec<Vec<u32>> = by_root.into_values().collect();
    out.sort_by_key(|g| g.first().copied());
    out
}

fn height(mesh: &Mesh, inc: &Incidence, facets: &[(Vec3, f64)], mask: &[bool], group: &[u32]) -> f64 {
    let verts: std::collections::BTreeSet<u32> =
        group.iter().filter_map(|&t| mesh.triangles.get(t as usize)).flatten().copied().collect();
    let ring: std::collections::BTreeSet<u32> = verts
        .iter()
        .flat_map(|&v| inc.row(v).iter().copied().take(MAX_ROW))
        .filter(|&t| !mask.get(t as usize).copied().unwrap_or(true))
        .collect();
    let (mut normal, mut centre, mut total) = (Vec3::zeros(), Vec3::zeros(), 0.0);
    for &t in &ring {
        if let (Some(&(n, a)), Some(c)) = (facets.get(t as usize), mesh.face_centroid(t as usize)) {
            normal += n * a;
            centre += c.coords * a;
            total += a;
        }
    }
    let Some(unit) = crate::normalize(normal) else { return f64::INFINITY };
    if total <= 0.0 {
        return f64::INFINITY;
    }
    let c = Point3::from(centre / total);
    verts.iter().filter_map(|&v| mesh.positions.get(v as usize)).map(|p| (p - c).dot(&unit).abs()).fold(0.0, f64::max)
}

/// The groups of flipped triangles of `mesh` with their measures.
pub(super) fn sites(mesh: &Mesh, inc: &Incidence, facets: &[(Vec3, f64)], mask: &[bool]) -> Vec<Site> {
    let neighbours = mesh.triangle_neighbours();
    let bvh = Bvh::build(mesh);
    par::map_slice(&groups(mesh, mask), |g| {
        let corners = g.iter().filter_map(|&t| mesh.triangle(t as usize)).flatten();
        let extent = Aabb::from_points(corners.collect::<Vec<_>>().iter()).diagonal();
        let area = g.iter().map(|&t| facets.get(t as usize).map_or(0.0, |f| f.1)).sum();
        let on_rim = g.iter().any(|&t| neighbours.get(t as usize).is_some_and(|n| n.contains(&u32::MAX)));
        Site {
            triangles: g.clone(),
            area,
            extent,
            height: height(mesh, inc, facets, mask, g),
            on_rim,
            crossing: g.iter().any(|&t| crosses(mesh, &bvh, t as usize)),
        }
    })
}

/// Median edge length over an even sample of the triangles (0 for an empty mesh).
pub(super) fn median_edge(mesh: &Mesh) -> f64 {
    let n = mesh.triangles.len();
    let count = n.min(EDGE_SAMPLE_TRIANGLES);
    let mut lens: Vec<f64> = (0..count)
        .filter_map(|k| mesh.triangle(crate::even_index(k, n, count)))
        .flat_map(|[a, b, c]| [tri_len(&a, &b), tri_len(&b, &c), tri_len(&c, &a)])
        .collect();
    lens.sort_by(f64::total_cmp);
    lens.get(lens.len() / 2).copied().unwrap_or(0.0)
}

fn tri_len(a: &Point3, b: &Point3) -> f64 {
    (a - b).norm()
}

/// Do an edge of `a` pass through `b` (parameter within the edge)?
fn pierces(a: &[Point3; 3], b: &[Point3; 3]) -> bool {
    (0..3).any(|k| {
        let (p, q) = (a[k], a[(k + 1) % 3]);
        tri::ray_hit(&p, &(q - p), &b[0], &b[1], &b[2]).is_some_and(|u| u <= 1.0)
    })
}

/// The corners of a triangle except `skip` (the edge opposite it), as a pair of points.
fn opposite_edge(c: &[Point3; 3], ids: &[u32; 3], skip: u32) -> Option<(Point3, Point3)> {
    let k = ids.iter().position(|&v| v == skip)?;
    Some((c[(k + 1) % 3], c[(k + 2) % 3]))
}

fn edge_pierces(edge: Option<(Point3, Point3)>, b: &[Point3; 3]) -> bool {
    edge.is_some_and(|(p, q)| tri::ray_hit(&p, &(q - p), &b[0], &b[1], &b[2]).is_some_and(|u| u <= 1.0))
}

/// Are `x` and `y`, which share the edge `(a, b)`, folded flat onto each other: normals opposite and the
/// third corners on the same side of the edge? (A triangle that is only wound the wrong way lies on the
/// other side of the edge and is no fold.)
fn folded_flat(x: (&[Point3; 3], &[u32; 3]), y: (&[Point3; 3], &[u32; 3]), (a, b): (u32, u32)) -> bool {
    let apex =
        |t: (&[Point3; 3], &[u32; 3])| t.1.iter().position(|v| *v != a && *v != b).and_then(|k| t.0.get(k).copied());
    let edge =
        |t: (&[Point3; 3], &[u32; 3]), v: u32| t.1.iter().position(|w| *w == v).and_then(|k| t.0.get(k).copied());
    let (Some(pa), Some(pb), Some(e0), Some(e1)) = (apex(x), apex(y), edge(x, a), edge(x, b)) else { return false };
    let along = e1 - e0;
    let same_side = along.cross(&(pa - e0)).dot(&along.cross(&(pb - e0))) > 0.0;
    let opposite = match (tri::normal(&x.0[0], &x.0[1], &x.0[2]), tri::normal(&y.0[0], &y.0[1], &y.0[2])) {
        (Some(n), Some(m)) => n.dot(&m) < -FOLDED_FLAT_COS,
        _ => false,
    };
    same_side && opposite
}

/// Do the triangles `x` and `y` (corners, vertex ids) pass through each other? Triangles that share
/// nothing: an edge of one pierces the other. That share a vertex: the edge opposite that vertex pierces
/// the other (an edge from the shared vertex cannot, unless the two lie in one plane). That share an edge:
/// folded flat onto each other.
fn cross(x: (&[Point3; 3], &[u32; 3]), y: (&[Point3; 3], &[u32; 3])) -> bool {
    let shared: Vec<u32> = x.1.iter().copied().filter(|v| y.1.contains(v)).collect();
    match shared.as_slice() {
        [] => pierces(x.0, y.0) || pierces(y.0, x.0),
        [v] => edge_pierces(opposite_edge(x.0, x.1, *v), y.0) || edge_pierces(opposite_edge(y.0, y.1, *v), x.0),
        [a, b] => folded_flat(x, y, (*a, *b)),
        _ => false,
    }
}

/// Does triangle `t` pass through another triangle (see [`cross`])?
fn crosses(mesh: &Mesh, bvh: &Bvh, t: usize) -> bool {
    let (Some(corners), Some(own)) = (mesh.triangle(t), mesh.triangles.get(t)) else { return false };
    let query = Aabb::from_points(corners.iter());
    let (others, _) = bvh.overlapping_up_to(&query, MAX_CANDIDATES, &|_| true);
    others.iter().filter(|&&o| o as usize != t).any(|&o| {
        let (Some(theirs), Some(ids)) = (mesh.triangle(o as usize), mesh.triangles.get(o as usize)) else {
            return false;
        };
        cross((&corners, own), (&theirs, ids))
    })
}
