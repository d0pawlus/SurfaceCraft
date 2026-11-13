//! The patch cap of `surface.wrap`: a coarse mesh is brought under `max_patches / 3` triangles, and
//! a refinement that would not fit is searched for the largest one that does.
//!
//! Edge collapse alone cannot always reach a count: a scan whose surface carries thousands of tiny
//! handles and thin bridges keeps them, because collapsing one would change the topology (the
//! reduction of one real scan stopped at 30 000 triangles for a limit of 4 000). Then the mesh is
//! clustered on a grid, which has no topology to keep: the cell is widened until the count fits.
use super::wrap_cluster::{ClusterError, cluster_to_cap};
use surfacecraft_geom::Mesh;
use surfacecraft_geom::edit::{DecimateOptions, decimate, remove_non_manifold};

/// A search ends when a mesh holds at least this share of the triangles the cap allows.
pub(super) const FILL_SHARE: f64 = 0.9;
/// A mesh up to this much over the cap is trimmed to it by the decimation instead of being searched
/// for again.
const TRIM_SLACK: f64 = 0.1;
/// Reductions a search runs at most (each is a full reduction of the scan).
pub(super) const MAX_PROBES: usize = 4;
/// The share of a bracket a probe may move from either end (the search always makes progress).
const MIN_STEP: f64 = 0.1;

pub(super) fn decimate_options() -> DecimateOptions {
    DecimateOptions { target_triangles: 0, max_error: None, feature_angle_deg: 181.0, preserve_boundary: true }
}

/// A mesh within the cap, and whether clustering made it.
pub(super) struct Trimmed {
    pub(super) mesh: Mesh,
    pub(super) clustered: bool,
}

/// `mesh` made manifold and cut to at most `cap` triangles: by the quadric decimation, and when that
/// stops short (`allow_cluster`) by clustering ([`cluster_to_cap`]). The error says why no mesh
/// within the cap could be made (`Over`: none exists, `Thin`: clustering would lose the surface).
pub(super) fn trim_to_cap(mut mesh: Mesh, cap: usize, allow_cluster: bool) -> Result<Trimmed, ClusterError> {
    remove_non_manifold(&mut mesh);
    if mesh.triangle_count() > cap {
        decimate(&mut mesh, &DecimateOptions { target_triangles: cap, ..decimate_options() });
        remove_non_manifold(&mut mesh);
    }
    if mesh.triangle_count() <= cap {
        return Ok(Trimmed { mesh, clustered: false });
    }
    if !allow_cluster {
        return Err(ClusterError::Over);
    }
    cluster_to_cap(&mesh, cap).map(|mesh| Trimmed { mesh, clustered: true })
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
            c if c as f64 <= cap as f64 * (1.0 + TRIM_SLACK) => trim_to_cap(mesh, cap, false).ok().map(|t| t.mesh),
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
    let last_resort =
        || smallest_over.and_then(|o| trim_to_cap(o.mesh, cap, false).ok().map(|t| Found { x: o.x, mesh: t.mesh }));
    (best.or_else(last_resort), probes)
}

#[cfg(test)]
mod tests {
    use super::*;
    use surfacecraft_geom::{Point3, synth};

    fn sphere(res: usize) -> Mesh {
        synth::sphere(Point3::origin(), 10.0, res, res / 2)
    }

    /// A sphere (which edge collapse reduces well) among a thousand small closed shells, each a
    /// tetrahedron of four triangles: no edge of a tetrahedron can be collapsed (link condition), so
    /// decimation stalls at about four thousand triangles, the way a scan with many small handles and
    /// loops stalls.
    fn sphere_with_debris() -> Mesh {
        let mut m = sphere(40);
        for i in -5..=5 {
            for j in -5..=5 {
                for k in -5..=5 {
                    let c = Point3::new(6.0 * f64::from(i), 6.0 * f64::from(j), 6.0 * f64::from(k));
                    if c.coords.norm() < 14.0 {
                        continue;
                    }
                    let base = m.positions.len() as u32;
                    let corners = [(0.0, 0.0, 0.0), (0.8, 0.0, 0.0), (0.0, 0.8, 0.0), (0.0, 0.0, 0.8)];
                    m.positions.extend(corners.iter().map(|d| Point3::new(c.x + d.0, c.y + d.1, c.z + d.2)));
                    m.triangles.extend([[0, 2, 1], [0, 1, 3], [1, 2, 3], [0, 3, 2]].map(|t| t.map(|v| base + v)));
                }
            }
        }
        m
    }

    #[test]
    fn a_mesh_decimation_cannot_reduce_is_clustered_under_the_cap() {
        // Arrange
        let mesh = sphere_with_debris();
        let cap = 2000;
        assert!(mesh.triangle_count() > 2 * cap);
        let stalled = trim_to_cap(mesh.clone(), cap, false);
        assert_eq!(stalled.err(), Some(ClusterError::Over), "decimation alone must stall on this mesh");

        // Act
        let out = trim_to_cap(mesh, cap, true).unwrap();

        // Assert
        assert!(out.clustered);
        assert!(out.mesh.triangle_count() <= cap && out.mesh.triangle_count() >= 200, "{}", out.mesh.triangle_count());
    }

    #[test]
    fn trimming_lands_on_the_cap_and_keeps_a_smaller_mesh_as_it_is() {
        // Arrange
        let mesh = sphere(60);
        let small = sphere(8);

        // Act
        let cut = trim_to_cap(mesh.clone(), 500, false).unwrap().mesh;
        let same = trim_to_cap(small.clone(), 500, false).unwrap().mesh;

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
