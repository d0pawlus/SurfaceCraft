//! Topology and sizing checks of a mesh before modelling (scan quality, P8-E2): how edges are
//! shared by triangles, degenerate and duplicate triangles, edge lengths. Pure functions of the
//! mesh: no state and no file access.
use crate::{Mesh, tri};

/// Triangles with an area at or below this (mm²), or with a repeated corner, are degenerate.
pub const DEGENERATE_AREA: f64 = 1e-9;

/// Summary of a list of lengths (mm); all zero for an empty list.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct LengthStats {
    /// Shortest.
    pub min: f64,
    /// Median.
    pub median: f64,
    /// Mean.
    pub mean: f64,
    /// Longest.
    pub max: f64,
}

/// How the distinct edges of a mesh are shared by its triangles.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct EdgeCensus {
    /// Distinct undirected edges.
    pub edges: usize,
    /// Edges used by exactly one triangle: the rim of an open surface.
    pub boundary: usize,
    /// Edges used by three or more triangles.
    pub non_manifold: usize,
    /// Lengths of the distinct edges.
    pub lengths: LengthStats,
}

/// Edge census of `mesh`. A mesh that does not validate has none.
#[must_use]
pub fn edge_census(mesh: &Mesh) -> EdgeCensus {
    if mesh.validate().is_err() {
        return EdgeCensus::default();
    }
    let mut keys: Vec<u64> = Vec::with_capacity(mesh.triangles.len() * 3);
    for t in &mesh.triangles {
        for (a, b) in [(t[0], t[1]), (t[1], t[2]), (t[2], t[0])] {
            if a != b {
                keys.push(edge_key(a, b));
            }
        }
    }
    keys.sort_unstable();
    let mut census = EdgeCensus::default();
    let mut lengths = Vec::new();
    let mut start = 0;
    while let Some(&key) = keys.get(start) {
        let run = keys.get(start..).map_or(0, |rest| rest.iter().take_while(|&&k| k == key).count());
        census.edges += 1;
        match run {
            1 => census.boundary += 1,
            2 => {}
            _ => census.non_manifold += 1,
        }
        lengths.push(edge_length(mesh, key));
        start += run;
    }
    census.lengths = length_stats(lengths);
    census
}

/// Triangles that are flat (area at most [`DEGENERATE_AREA`]) or repeat a corner.
#[must_use]
pub fn degenerate_count(mesh: &Mesh) -> usize {
    mesh.triangles.iter().filter(|t| t[0] == t[1] || t[1] == t[2] || t[2] == t[0] || is_flat(mesh, t)).count()
}

/// Triangles that repeat the vertex set of an earlier triangle (orientation ignored).
#[must_use]
pub fn duplicate_count(mesh: &Mesh) -> usize {
    let mut sets: Vec<[u32; 3]> = mesh
        .triangles
        .iter()
        .map(|t| {
            let mut s = *t;
            s.sort_unstable();
            s
        })
        .collect();
    sets.sort_unstable();
    sets.windows(2).filter(|w| matches!(w, [a, b] if a == b)).count()
}

/// The undirected edge `a`–`b` as one key (smaller index in the high half).
fn edge_key(a: u32, b: u32) -> u64 {
    (u64::from(a.min(b)) << 32) | u64::from(a.max(b))
}

/// Length of the edge with key `key` (zero when a corner is missing).
fn edge_length(mesh: &Mesh, key: u64) -> f64 {
    let a = (key >> 32) as usize;
    let b = (key & u64::from(u32::MAX)) as usize;
    match (mesh.positions.get(a), mesh.positions.get(b)) {
        (Some(p), Some(q)) => (p - q).norm(),
        _ => 0.0,
    }
}

/// Whether triangle `t` has an area at most [`DEGENERATE_AREA`] (or a corner that is missing).
fn is_flat(mesh: &Mesh, t: &[u32; 3]) -> bool {
    let corner = |i: u32| mesh.positions.get(i as usize);
    match (corner(t[0]), corner(t[1]), corner(t[2])) {
        (Some(a), Some(b), Some(c)) => !(tri::area(a, b, c) > DEGENERATE_AREA),
        _ => true,
    }
}

/// Summary of `lengths`: sorted in place, zero when empty.
fn length_stats(mut lengths: Vec<f64>) -> LengthStats {
    lengths.sort_unstable_by(f64::total_cmp);
    let n = lengths.len();
    let (Some(&min), Some(&max)) = (lengths.first(), lengths.last()) else {
        return LengthStats::default();
    };
    let mid = n / 2;
    let median = if n % 2 == 1 {
        lengths.get(mid).copied().unwrap_or(0.0)
    } else {
        let hi = lengths.get(mid).copied().unwrap_or(0.0);
        let lo = lengths.get(mid.saturating_sub(1)).copied().unwrap_or(0.0);
        (lo + hi) / 2.0
    };
    LengthStats { min, median, mean: lengths.iter().sum::<f64>() / n as f64, max }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The tetrahedron with corners at the origin and on the three unit axes.
    fn tetrahedron() -> Mesh {
        let p = vec![
            crate::Point3::origin(),
            crate::Point3::new(1.0, 0.0, 0.0),
            crate::Point3::new(0.0, 1.0, 0.0),
            crate::Point3::new(0.0, 0.0, 1.0),
        ];
        Mesh::new(p, vec![[0, 2, 1], [0, 1, 3], [0, 3, 2], [1, 2, 3]]).unwrap()
    }

    #[test]
    fn closed_tetrahedron_is_manifold_with_axis_and_diagonal_edges() {
        let c = edge_census(&tetrahedron());
        assert_eq!(c.edges, 6);
        assert_eq!((c.boundary, c.non_manifold), (0, 0));
        assert!((c.lengths.min - 1.0).abs() < 1e-12, "{c:?}");
        assert!((c.lengths.max - 2f64.sqrt()).abs() < 1e-12, "{c:?}");
    }

    #[test]
    fn open_quad_has_a_rim_of_four_edges() {
        let p = (0..4).map(|i| crate::Point3::new(f64::from(i % 2), f64::from(i / 2), 0.0)).collect();
        let m = Mesh::new(p, vec![[0, 1, 3], [0, 3, 2]]).unwrap();
        let c = edge_census(&m);
        assert_eq!(c.edges, 5, "{c:?}");
        assert_eq!((c.boundary, c.non_manifold), (4, 0), "{c:?}");
    }

    #[test]
    fn third_triangle_on_one_edge_is_non_manifold() {
        let p = (0..5).map(|i| crate::Point3::new(f64::from(i), f64::from(i * i % 3), 1.0)).collect();
        let m = Mesh::new(p, vec![[0, 1, 2], [1, 0, 3], [0, 1, 4]]).unwrap();
        let c = edge_census(&m);
        assert_eq!(c.edges, 7, "{c:?}");
        assert_eq!((c.boundary, c.non_manifold), (6, 1), "{c:?}");
    }

    #[test]
    fn flat_and_repeated_triangles_are_degenerate_and_duplicates_are_counted() {
        let p = vec![
            crate::Point3::new(0.0, 0.0, 0.0),
            crate::Point3::new(1.0, 0.0, 0.0),
            crate::Point3::new(2.0, 0.0, 0.0),
            crate::Point3::new(0.0, 1.0, 0.0),
        ];
        let m = Mesh::new(p, vec![[0, 1, 2], [0, 2, 1], [0, 1, 3]]).unwrap();
        assert_eq!(degenerate_count(&m), 2);
        assert_eq!(duplicate_count(&m), 1);
    }

    #[test]
    fn triangle_with_a_repeated_corner_is_degenerate() {
        let p = vec![crate::Point3::origin(), crate::Point3::new(1.0, 0.0, 0.0), crate::Point3::new(0.0, 1.0, 0.0)];
        let m = Mesh::new(p, vec![[0, 0, 1], [0, 1, 2]]).unwrap();
        assert_eq!(degenerate_count(&m), 1);
    }

    #[test]
    fn empty_mesh_has_no_edges_and_no_lengths() {
        assert_eq!(edge_census(&Mesh::default()), EdgeCensus::default());
    }
}
