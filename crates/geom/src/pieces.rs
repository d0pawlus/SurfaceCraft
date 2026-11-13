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
}

impl Piece {
    /// No boundary and no non-manifold edge: a watertight surface.
    #[must_use]
    pub fn closed(&self) -> bool {
        self.boundary_edges == 0 && self.non_manifold_edges == 0
    }

    /// Genus of the surface assuming it is an orientable manifold with simple boundary loops;
    /// `None` when it has non-manifold edges or the Euler characteristic is inconsistent with that.
    #[must_use]
    pub fn genus(&self) -> Option<usize> {
        if self.non_manifold_edges > 0 {
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

/// Edge facts of the whole mesh, attributed to the piece of the first triangle of each edge:
/// `(boundary edges as (a, b, piece), non-manifold edges per piece, distinct edges per piece)`.
fn edge_facts(mesh: &Mesh, piece_of: &[u32], pieces: usize) -> (Vec<(u32, u32, u32)>, Vec<usize>, Vec<usize>) {
    let mut half: Vec<(u64, u32)> = Vec::with_capacity(mesh.triangles.len() * 3);
    for (ti, t) in mesh.triangles.iter().enumerate() {
        for k in 0..3 {
            let (a, b) = (t[k], t[(k + 1) % 3]);
            if a != b {
                half.push((edge_key(a, b), ti as u32));
            }
        }
    }
    par::sort_unstable(&mut half);
    let (mut boundary, mut non_manifold, mut edges) = (Vec::new(), vec![0usize; pieces], vec![0usize; pieces]);
    let mut i = 0;
    while let Some(&(key, tri)) = half.get(i) {
        let run = half.get(i..).map_or(0, |r| r.iter().take_while(|h| h.0 == key).count());
        let piece = piece_of.get(tri as usize).copied().unwrap_or(0);
        if let Some(e) = edges.get_mut(piece as usize) {
            *e += 1;
        }
        match run {
            1 => boundary.push(((key >> 32) as u32, (key & 0xffff_ffff) as u32, piece)),
            2 => {}
            _ => {
                if let Some(n) = non_manifold.get_mut(piece as usize) {
                    *n += 1;
                }
            }
        }
        i += run;
    }
    (boundary, non_manifold, edges)
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

/// The connected pieces of `mesh`, most triangles first (ties: lowest triangle index first).
/// An invalid mesh has none.
#[must_use]
pub fn pieces(mesh: &Mesh) -> Vec<Piece> {
    let groups = edit::components(mesh);
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
    let (boundary, non_manifold, edges) = edge_facts(mesh, &piece_of, groups.len());
    let mut loops = loops_of(mesh, &boundary, groups.len());
    let mut boundary_edges = vec![0usize; groups.len()];
    for &(_, _, p) in &boundary {
        if let Some(n) = boundary_edges.get_mut(p as usize) {
            *n += 1;
        }
    }
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
            let (v, e) = (vertices.get(pi).copied().unwrap_or(0), edges.get(pi).copied().unwrap_or(0));
            Piece {
                euler: v as i64 - e as i64 + triangles.len() as i64,
                vertices: v,
                area,
                bbox,
                boundary_edges: boundary_edges.get(pi).copied().unwrap_or(0),
                loops: loops.get_mut(pi).map(std::mem::take).unwrap_or_default(),
                non_manifold_edges: non_manifold.get(pi).copied().unwrap_or(0),
                edges: e,
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
