//! The patch cap of `surface.wrap`: a coarse mesh is brought under `max_patches / 3` triangles, and
//! a refinement that would not fit is searched for the largest one that does.
//!
//! Edge collapse alone cannot always reach a count: a scan whose surface carries thousands of tiny
//! handles and thin bridges keeps them, because collapsing one would change the topology (the
//! reduction of one real scan stopped at 30 000 triangles for a limit of 4 000). Then the mesh is
//! clustered on a grid, which has no topology to keep: the cell is widened until the count fits.
use std::collections::BTreeMap;
use surfacecraft_geom::edit::{DecimateOptions, decimate, remove_non_manifold};
use surfacecraft_geom::{Mesh, Point3};

/// A search ends when a mesh holds at least this share of the triangles the cap allows.
pub(super) const FILL_SHARE: f64 = 0.9;
/// A mesh up to this much over the cap is trimmed to it by the decimation instead of being searched
/// for again.
const TRIM_SLACK: f64 = 0.1;
/// Reductions a search runs at most (each is a full reduction of the scan).
pub(super) const MAX_PROBES: usize = 4;
/// Bisection steps of the cell size of the clustering.
const CLUSTER_STEPS: usize = 18;
/// The widest cell is the diagonal of the box; the finest is this fraction of it.
const FINEST_CELL: f64 = 1.0 / 4096.0;
/// The share of a bracket a probe may move from either end (the search always makes progress).
const MIN_STEP: f64 = 0.1;

pub(super) fn decimate_options() -> DecimateOptions {
    DecimateOptions { target_triangles: 0, max_error: None, feature_angle_deg: 181.0, preserve_boundary: true }
}

/// `mesh` made manifold and cut to at most `cap` triangles: by the quadric decimation, and when that
/// stops short (`allow_cluster`) by clustering. `None` when no mesh within the cap could be made.
pub(super) fn trim_to_cap(mut mesh: Mesh, cap: usize, allow_cluster: bool) -> Option<Mesh> {
    remove_non_manifold(&mut mesh);
    if mesh.triangle_count() <= cap {
        return Some(mesh);
    }
    decimate(&mut mesh, &DecimateOptions { target_triangles: cap, ..decimate_options() });
    remove_non_manifold(&mut mesh);
    if mesh.triangle_count() <= cap {
        return Some(mesh);
    }
    if allow_cluster { cluster_to_cap(&mesh, cap) } else { None }
}

/// The cell of a point: floor of its offset from `origin` in cells of size `cell`.
fn cell_of(p: &Point3, origin: &Point3, cell: f64) -> [i64; 3] {
    let k = |a: f64, o: f64| ((a - o) / cell).floor() as i64;
    [k(p.x, origin.x), k(p.y, origin.y), k(p.z, origin.z)]
}

/// The mesh with the vertices of every grid cell merged into their mean; triangles that lose a
/// corner and repeated triangles are dropped, and the rest is made manifold.
fn cluster(mesh: &Mesh, origin: &Point3, cell: f64) -> Mesh {
    let keys: Vec<[i64; 3]> = mesh.positions.iter().map(|p| cell_of(p, origin, cell)).collect();
    let mut index: BTreeMap<[i64; 3], usize> = BTreeMap::new();
    for k in mesh.triangles.iter().flatten().filter_map(|&v| keys.get(v as usize)) {
        index.entry(*k).or_insert(0);
    }
    for (n, slot) in index.values_mut().enumerate() {
        *slot = n;
    }
    let mut sum = vec![(0.0, 0.0, 0.0, 0usize); index.len()];
    let used: Vec<bool> = {
        let mut u = vec![false; mesh.positions.len()];
        mesh.triangles.iter().flatten().for_each(|&v| u.get_mut(v as usize).map_or((), |x| *x = true));
        u
    };
    for (v, p) in mesh.positions.iter().enumerate() {
        let slot = used.get(v).copied().unwrap_or(false).then(|| keys.get(v).and_then(|k| index.get(k))).flatten();
        if let Some(s) = slot.and_then(|&c| sum.get_mut(c)) {
            *s = (s.0 + p.x, s.1 + p.y, s.2 + p.z, s.3 + 1);
        }
    }
    let positions =
        sum.iter().map(|s| Point3::new(s.0 / s.3.max(1) as f64, s.1 / s.3.max(1) as f64, s.2 / s.3.max(1) as f64)).collect();
    let cell_id = |v: u32| keys.get(v as usize).and_then(|k| index.get(k)).map(|&c| c as u32);
    let mut seen = std::collections::HashSet::new();
    let mut triangles = Vec::new();
    for t in &mesh.triangles {
        let (Some(a), Some(b), Some(c)) = (cell_id(t[0]), cell_id(t[1]), cell_id(t[2])) else { continue };
        let mut sorted = [a, b, c];
        sorted.sort_unstable();
        if a != b && b != c && a != c && seen.insert(sorted) {
            triangles.push([a, b, c]);
        }
    }
    let mut out = Mesh { positions, triangles };
    remove_non_manifold(&mut out);
    out
}

/// The clustering of `mesh` with the finest cell that leaves at most `cap` triangles (bisection of
/// the cell size, [`CLUSTER_STEPS`] steps), or `None` when even the widest cell leaves more.
fn cluster_to_cap(mesh: &Mesh, cap: usize) -> Option<Mesh> {
    let bbox = mesh.bbox();
    if bbox.is_empty() {
        return None;
    }
    let diagonal = (bbox.max - bbox.min).norm().max(1e-9);
    let (mut fine, mut wide) = (diagonal * FINEST_CELL, diagonal);
    let mut best = Some(cluster(mesh, &bbox.min, wide)).filter(|m| m.triangle_count() <= cap)?;
    for _ in 0..CLUSTER_STEPS {
        let mid = (fine * wide).sqrt();
        let m = cluster(mesh, &bbox.min, mid);
        if m.triangle_count() <= cap {
            wide = mid;
            if m.triangle_count() >= best.triangle_count() {
                best = m;
            }
        } else {
            fine = mid;
        }
    }
    Some(best)
}

/// One end of a bracket: the search parameter and the triangles of the mesh it gives.
#[derive(Clone, Copy, Debug)]
pub(super) struct End {
    pub(super) x: f64,
    pub(super) triangles: usize,
}

/// What a search found: the parameter and its mesh.
pub(super) struct Found {
    pub(super) x: f64,
    pub(super) mesh: Mesh,
}

/// The parameter between the ends where the triangle count is expected to reach `target` (linear in
/// the logarithm of the count), kept a step away from both ends.
fn next_probe(lo: End, hi: End, target: f64) -> f64 {
    let (a, b) = ((lo.triangles.max(1) as f64).ln(), (hi.triangles.max(1) as f64).ln());
    let share = if b > a { ((target.ln() - a) / (b - a)).clamp(MIN_STEP, 1.0 - MIN_STEP) } else { 0.5 };
    lo.x + share * (hi.x - lo.x)
}

/// Searches the parameter `x` between `lo` (a mesh of at most `cap` triangles) and `hi` (more than
/// `cap`) for the mesh closest under the cap, with at most [`MAX_PROBES`] calls of `probe`; the
/// triangle count must grow with `x`. A probe over the cap by at most [`TRIM_SLACK`] is cut to the
/// cap by the decimation. The search ends as soon as a mesh holds [`FILL_SHARE`] of the cap, so the
/// result has between 90 % and 100 % of the cap when a probe lands there. `step` is the smallest
/// distance between two probes in `x` (1 for integer parameters). Also returns the probes made; the
/// mesh is `None` when no probe fitted.
pub(super) fn search_cap(
    cap: usize,
    (mut lo, mut hi): (End, End),
    step: f64,
    mut probe: impl FnMut(f64) -> Mesh,
) -> (Option<Found>, usize) {
    let mut best: Option<Found> = None;
    // the smallest probe over the cap by more than the slack: the last resort
    let mut smallest_over: Option<Found> = None;
    let mut probes = 0;
    let target = cap as f64 * (1.0 + 0.5 * TRIM_SLACK);
    while probes < MAX_PROBES {
        let x = next_probe(lo, hi, target);
        if x - lo.x < step || hi.x - x < step {
            break;
        }
        probes += 1;
        let mesh = probe(x);
        let count = mesh.triangle_count();
        let fitted = match count {
            c if c <= cap => Some(mesh),
            c if c as f64 <= cap as f64 * (1.0 + TRIM_SLACK) => trim_to_cap(mesh, cap, false),
            _ => {
                if smallest_over.as_ref().is_none_or(|o| count < o.mesh.triangle_count()) {
                    smallest_over = Some(Found { x, mesh });
                }
                None
            }
        };
        if count <= cap {
            lo = End { x, triangles: count };
        } else {
            hi = End { x, triangles: count };
        }
        if let Some(mesh) = fitted
            && best.as_ref().is_none_or(|b| mesh.triangle_count() > b.mesh.triangle_count())
        {
            best = Some(Found { x, mesh });
        }
        if best.as_ref().is_some_and(|b| b.mesh.triangle_count() as f64 >= FILL_SHARE * cap as f64) {
            break;
        }
    }
    // every probe was far over the cap: cut the closest one to it
    let last_resort = || smallest_over.and_then(|o| trim_to_cap(o.mesh, cap, false).map(|mesh| Found { x: o.x, mesh }));
    (best.or_else(last_resort), probes)
}

#[cfg(test)]
mod tests {
    use super::*;
    use surfacecraft_geom::synth;

    fn sphere(res: usize) -> Mesh {
        synth::sphere(Point3::origin(), 10.0, res, res / 2)
    }

    #[test]
    fn clustering_reaches_a_cap_the_mesh_has_many_times_over() {
        // Arrange
        let mesh = sphere(120);
        assert!(mesh.triangle_count() > 10_000);

        // Act
        let out = cluster_to_cap(&mesh, 300).unwrap();

        // Assert
        assert!(out.triangle_count() <= 300, "{}", out.triangle_count());
        assert!(out.triangle_count() >= 150, "{}", out.triangle_count());
        assert!(out.validate().is_ok());
        assert_eq!(out, cluster_to_cap(&mesh, 300).unwrap());
    }

    #[test]
    fn clustering_has_no_answer_for_an_empty_mesh() {
        assert!(cluster_to_cap(&Mesh::default(), 10).is_none());
    }

    #[test]
    fn trimming_lands_on_the_cap_and_keeps_a_smaller_mesh_as_it_is() {
        // Arrange
        let mesh = sphere(60);
        let small = sphere(8);

        // Act
        let cut = trim_to_cap(mesh.clone(), 500, false).unwrap();
        let same = trim_to_cap(small.clone(), 500, false).unwrap();

        // Assert
        assert!(cut.triangle_count() <= 500 && cut.triangle_count() >= 490, "{}", cut.triangle_count());
        assert_eq!(same.triangle_count(), small.triangle_count());
    }

    /// A probe whose triangle count grows with `x` as `100 * x^2`.
    fn quadratic(x: f64) -> Mesh {
        let n = (100.0 * x * x) as usize;
        let mut m = synth::sphere(Point3::origin(), 1.0, 40, 20);
        m.triangles.truncate(n);
        m
    }

    #[test]
    fn the_search_lands_in_the_last_tenth_under_the_cap_within_the_probe_limit() {
        // Arrange: counts 100 at x = 1 and 400 at x = 2; the cap 250 is at x = 1.58
        let (lo, hi) = (End { x: 1.0, triangles: 100 }, End { x: 2.0, triangles: 400 });
        let mut calls = 0;

        // Act
        let (found, probes) = search_cap(250, (lo, hi), 0.001, |x| {
            calls += 1;
            quadratic(x)
        });

        // Assert
        let found = found.unwrap();
        let count = found.mesh.triangle_count();
        assert!((225..=250).contains(&count), "{count}");
        assert!(probes <= MAX_PROBES && probes == calls);
    }

    #[test]
    fn the_search_returns_nothing_when_no_probe_can_be_cut_to_the_cap() {
        // Arrange: every probe is far over the cap and cannot be decimated (a single sheet of 2 triangles
        // is the floor), so nothing fits
        let (lo, hi) = (End { x: 0.0, triangles: 1 }, End { x: 1.0, triangles: 10_000 });

        // Act
        let (found, probes) = search_cap(0, (lo, hi), 0.001, |_| sphere(20));

        // Assert
        assert!(found.is_none());
        assert!(probes >= 1);
    }
}
