//! The connected pieces of a scan and what is known about each: edges, boundary loops, Euler
//! characteristic and genus (design: P9-O9). A piece is a set of triangles connected through shared
//! vertices; the facts are the ones that tell "one part" from "several parts or shells in one file"
//! and "a clean surface" from "a surface with topological noise".
use crate::{Aabb, Mesh, edit, par};

/// A closed curve of boundary edges (edges of one triangle only).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BoundaryLoop {
    /// Edges of the loop (a union of boundary edges sharing vertices; a pinch counts both rims).
    pub edges: usize,
    /// Summed edge length (mm).
    pub length: f64,
}

/// One connected piece of a mesh.
#[derive(Clone, Debug)]
pub struct Piece {
    /// Triangles of the piece (ascending indices into the mesh).
    pub triangles: Vec<u32>,
    /// Vertices the triangles use.
    pub vertices: usize,
    /// Surface area (mm²).
    pub area: f64,
    /// Bounding box.
    pub bbox: Aabb,
    /// Edges of exactly one triangle.
    pub boundary_edges: usize,
    /// Boundary loops, the longest (in edges) first.
    pub loops: Vec<BoundaryLoop>,
    /// Edges of three or more triangles.
    pub non_manifold_edges: usize,
    /// Distinct edges.
    pub edges: usize,
    /// Euler characteristic `V − E + F`.
    pub euler: i64,
    /// Vertices whose triangles form more than one fan (pinch points).
    pub pinched_vertices: usize,
    /// Edges of two triangles that run the same way in both (inconsistent orientation, or not orientable).
    pub unoriented_edges: usize,
}

impl Piece {
    /// No boundary and no non-manifold edge: a watertight surface.
    #[must_use]
    pub fn closed(&self) -> bool {
        self.boundary_edges == 0 && self.non_manifold_edges == 0
    }

    /// Genus of the surface; `None` unless it is an oriented manifold (no non-manifold edge, no pinched
    /// vertex, no edge that runs the same way twice) with a consistent Euler characteristic.
    #[must_use]
    pub fn genus(&self) -> Option<usize> {
        if self.non_manifold_edges > 0 || self.pinched_vertices > 0 || self.unoriented_edges > 0 {
            return None;
        }
        let twice = 2 - self.loops.len() as i64 - self.euler;
        (twice >= 0 && twice % 2 == 0).then_some((twice / 2) as usize)
    }
}

fn edge_key(a: u32, b: u32) -> u64 {
    (u64::from(a.min(b)) << 32) | u64::from(a.max(b))
}

fn find(parent: &mut [u32], mut x: u32) -> u32 {
    while let Some(&p) = parent.get(x as usize) {
        if p == x {
            break;
        }
        let up = parent.get(p as usize).copied().unwrap_or(p);
        if let Some(slot) = parent.get_mut(x as usize) {
            *slot = up;
        }
        x = up;
    }
    x
}

/// Edge facts of the whole mesh, attributed to the piece of the first triangle of each edge.
struct EdgeFacts {
    /// Boundary edges as `(a, b, piece)`.
    boundary: Vec<(u32, u32, u32)>,
    non_manifold: Vec<usize>,
    edges: Vec<usize>,
    /// Edges of two triangles that run the same way in both: not an oriented surface.
    unoriented: Vec<usize>,
    /// Union-find over the corners `3 · triangle + k`: corners of one vertex joined across the edges
    /// of two triangles; more classes than vertices means a pinched vertex.
    corners: Vec<u32>,
}

fn corner_of(mesh: &Mesh, tri: u32, vertex: u32) -> u32 {
    let k = mesh.triangles.get(tri as usize).and_then(|t| t.iter().position(|v| *v == vertex)).unwrap_or(0);
    tri * 3 + k as u32
}

fn edge_facts(mesh: &Mesh, piece_of: &[u32], pieces: usize) -> EdgeFacts {
    let mut half: Vec<(u64, u32)> = Vec::with_capacity(mesh.triangles.len() * 3);
    for (ti, t) in mesh.triangles.iter().enumerate() {
        for k in 0..3 {
            let (a, b) = (t[k], t[(k + 1) % 3]);
            if a != b {
                half.push((edge_key(a, b), (ti as u32) << 1 | u32::from(a < b)));
            }
        }
    }
    par::sort_unstable(&mut half);
    let mut f = EdgeFacts {
        boundary: Vec::new(),
        non_manifold: vec![0; pieces],
        edges: vec![0; pieces],
        unoriented: vec![0; pieces],
        corners: (0..mesh.triangles.len() as u32 * 3).collect(),
    };
    let mut i = 0;
    while let Some(&(key, first)) = half.get(i) {
        let run = half.get(i..).map_or(0, |r| r.iter().take_while(|h| h.0 == key).count());
        let (a, b) = ((key >> 32) as u32, (key & 0xffff_ffff) as u32);
        let piece = piece_of.get((first >> 1) as usize).copied().unwrap_or(0) as usize;
        if let Some(e) = f.edges.get_mut(piece) {
            *e += 1;
        }
        match (run, half.get(i + 1)) {
            (1, _) => f.boundary.push((a, b, piece as u32)),
            (2, Some(&(_, second))) => {
                if (first & 1) == (second & 1)
                    && let Some(u) = f.unoriented.get_mut(piece)
                {
                    *u += 1;
                }
                for v in [a, b] {
                    let (x, y) = (corner_of(mesh, first >> 1, v), corner_of(mesh, second >> 1, v));
                    let (rx, ry) = (find(&mut f.corners, x), find(&mut f.corners, y));
                    if rx != ry
                        && let Some(slot) = f.corners.get_mut(ry as usize)
                    {
                        *slot = rx;
                    }
                }
            }
            _ => {
                if let Some(n) = f.non_manifold.get_mut(piece) {
                    *n += 1;
                }
            }
        }
        i += run;
    }
    f
}

/// Vertices per piece and the pinched ones (corner classes beyond one per vertex).
fn pinches(piece_of: &[u32], vertices: &[usize], corners: &mut [u32]) -> Vec<usize> {
    let mut classes = vec![0usize; vertices.len()];
    for c in 0..corners.len() as u32 {
        if find(corners, c) == c
            && let Some(n) = piece_of.get((c / 3) as usize).and_then(|p| classes.get_mut(*p as usize))
        {
            *n += 1;
        }
    }
    classes.iter().zip(vertices).map(|(c, v)| c.saturating_sub(*v)).collect()
}

/// Boundary loops per piece: boundary edges that share vertices belong to one loop.
fn loops_of(mesh: &Mesh, boundary: &[(u32, u32, u32)], pieces: usize) -> Vec<Vec<BoundaryLoop>> {
    let mut parent: Vec<u32> = (0..mesh.positions.len() as u32).collect();
    for &(a, b, _) in boundary {
        let (ra, rb) = (find(&mut parent, a), find(&mut parent, b));
        if ra != rb
            && let Some(slot) = parent.get_mut(rb as usize)
        {
            *slot = ra;
        }
    }
    let mut by_root: std::collections::BTreeMap<u32, (u32, usize, f64)> = std::collections::BTreeMap::new();
    for &(a, b, piece) in boundary {
        let root = find(&mut parent, a);
        let len = match (mesh.positions.get(a as usize), mesh.positions.get(b as usize)) {
            (Some(p), Some(q)) => (p - q).norm(),
            _ => 0.0,
        };
        let entry = by_root.entry(root).or_insert((piece, 0, 0.0));
        entry.1 += 1;
        entry.2 += len;
    }
    let mut out: Vec<Vec<BoundaryLoop>> = vec![Vec::new(); pieces];
    for (_, (piece, edges, length)) in by_root {
        if let Some(list) = out.get_mut(piece as usize) {
            list.push(BoundaryLoop { edges, length });
        }
    }
    for list in &mut out {
        list.sort_by(|a, b| b.edges.cmp(&a.edges).then(b.length.total_cmp(&a.length)));
    }
    out
}

/// Which piece every triangle and vertex belongs to, and the vertices per piece.
fn assign(mesh: &Mesh, groups: &[Vec<u32>]) -> (Vec<u32>, Vec<usize>) {
    let mut piece_of = vec![0u32; mesh.triangles.len()];
    let mut vertex_piece = vec![u32::MAX; mesh.positions.len()];
    for (pi, g) in groups.iter().enumerate() {
        for &t in g {
            if let Some(slot) = piece_of.get_mut(t as usize) {
                *slot = pi as u32;
            }
            for v in mesh.triangles.get(t as usize).into_iter().flatten() {
                if let Some(slot) = vertex_piece.get_mut(*v as usize) {
                    *slot = pi as u32;
                }
            }
        }
    }
    let mut vertices = vec![0usize; groups.len()];
    for &p in &vertex_piece {
        if let Some(n) = vertices.get_mut(p as usize) {
            *n += 1;
        }
    }
    (piece_of, vertices)
}

/// The connected pieces of `mesh`, most triangles first (ties: lowest triangle index first).
/// An invalid mesh has none.
#[must_use]
pub fn pieces(mesh: &Mesh) -> Vec<Piece> {
    let groups = edit::components(mesh);
    let (piece_of, vertices) = assign(mesh, &groups);
    let mut facts = edge_facts(mesh, &piece_of, groups.len());
    let pinched = pinches(&piece_of, &vertices, &mut facts.corners);
    let mut loops = loops_of(mesh, &facts.boundary, groups.len());
    let mut boundary_edges = vec![0usize; groups.len()];
    for &(_, _, p) in &facts.boundary {
        if let Some(n) = boundary_edges.get_mut(p as usize) {
            *n += 1;
        }
    }
    let at = |v: &[usize], i: usize| v.get(i).copied().unwrap_or(0);
    groups
        .into_iter()
        .enumerate()
        .map(|(pi, triangles)| {
            let area = triangles.iter().map(|&t| mesh.face_area(t as usize)).sum();
            let bbox = Aabb::from_points(
                triangles
                    .iter()
                    .filter_map(|&t| mesh.triangles.get(t as usize))
                    .flatten()
                    .filter_map(|&v| mesh.positions.get(v as usize)),
            );
            let (v, e) = (at(&vertices, pi), at(&facts.edges, pi));
            Piece {
                euler: v as i64 - e as i64 + triangles.len() as i64,
                vertices: v,
                area,
                bbox,
                boundary_edges: at(&boundary_edges, pi),
                loops: loops.get_mut(pi).map(std::mem::take).unwrap_or_default(),
                non_manifold_edges: at(&facts.non_manifold, pi),
                edges: e,
                pinched_vertices: at(&pinched, pi),
                unoriented_edges: at(&facts.unoriented, pi),
                triangles,
            }
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{Point3, Vec3, synth};

    #[test]
    fn a_sphere_is_closed_with_genus_zero_and_a_torus_with_genus_one() {
        let mut sphere = synth::sphere(Point3::origin(), 5.0, 24, 12);
        sphere.weld(1e-9);
        let p = pieces(&sphere);
        assert_eq!(p.len(), 1);
        assert!(p[0].closed(), "{:?}", p[0].loops);
        assert_eq!(p[0].genus(), Some(0));
        let mut torus = synth::torus(Point3::origin(), Vec3::z(), 10.0, 2.0, 24, 12);
        torus.weld(1e-9);
        let t = pieces(&torus);
        assert!(t[0].closed());
        assert_eq!(t[0].genus(), Some(1));
    }

    #[test]
    fn genus_is_none_for_pinched_non_manifold_and_inconsistently_oriented_pieces() {
        let sphere = {
            let mut m = synth::sphere(Point3::origin(), 5.0, 24, 12);
            m.weld(1e-9);
            m
        };
        let mut flipped = sphere.clone();
        flipped.triangles[0].swap(1, 2);
        let p = &pieces(&flipped)[0];
        assert!(p.unoriented_edges > 0 && p.genus().is_none(), "{p:?}");
        // two spheres sharing one vertex: a pinch point
        let mut touching = sphere.clone();
        let mut other = sphere.clone();
        let top = sphere.positions.iter().map(|p| p.z).fold(f64::MIN, f64::max);
        let bottom = other.positions.iter().map(|p| p.z).fold(f64::MAX, f64::min);
        other.positions.iter_mut().for_each(|p| p.z += top - bottom);
        let base = touching.positions.len() as u32;
        touching.append(&other);
        let at_pole = |p: &Point3| (p.z - top).abs() < 1e-9 && p.x.abs() < 1e-9 && p.y.abs() < 1e-9;
        let pole_a = touching.positions.iter().position(at_pole).unwrap();
        let bottom_b = touching
            .positions
            .iter()
            .enumerate()
            .skip(base as usize)
            .find(|(_, p)| at_pole(p))
            .map(|(i, _)| i as u32)
            .unwrap();
        for t in touching.triangles.iter_mut().skip(sphere.triangles.len()) {
            t.iter_mut().filter(|v| **v == bottom_b).for_each(|v| *v = pole_a as u32);
        }
        let p = &pieces(&touching)[0];
        assert!(p.pinched_vertices > 0, "{p:?}");
        assert!(p.genus().is_none());
        // a fin: an edge of three triangles
        let mut finned = sphere;
        let n = finned.positions.len() as u32;
        finned.positions.push(Point3::new(0.0, 0.0, 9.0));
        let t = finned.triangles[0];
        finned.triangles.push([t[1], t[0], n]);
        assert!(pieces(&finned)[0].genus().is_none());
    }

    #[test]
    fn a_plane_has_one_boundary_loop_and_two_pieces_are_told_apart() {
        let mut m = synth::plane(Point3::origin(), Vec3::x(), Vec3::y(), 4.0, 4.0, 4, 4);
        m.weld(1e-9);
        let mut far = m.clone();
        for p in &mut far.positions {
            p.x += 100.0;
        }
        m.append(&far);
        let p = pieces(&m);
        assert_eq!(p.len(), 2);
        assert_eq!(p[0].loops.len(), 1);
        assert_eq!(p[0].loops[0].edges, 16);
        assert_eq!(p[0].genus(), Some(0));
        assert!((p[0].area - 16.0).abs() < 1e-9);
    }
}
