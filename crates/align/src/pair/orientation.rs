//! Is the orientation of the triangles trustworthy? The free-space conflicts ("in front of the
//! other scan's surface") and the descriptors read the side a triangle faces. A scan whose triangles
//! are wound against each other, or turned inside out, makes "in front" read "behind", which is
//! neutral: the conflicts vanish and a wrong pose could read clean. So every way the orientation can
//! be wrong that the meshes show makes the answer never `unique` (fail-closed):
//! - the winding of a scan is not consistent (neighbouring triangles traverse their shared edge in
//!   the same direction, or an edge is shared by more than two triangles: duplicated triangles);
//! - a closed scan has a negative signed volume (its triangles face inwards);
//! - at the pose, the normals of the two scans mostly oppose where their surfaces meet (one scan is
//!   turned inside out relative to the other).
//!
//! Not detectable from the meshes: two OPEN scans that are both inverted, consistently. Each is
//! wound consistently, none is closed, and their normals agree at every pose: the conflicts then
//! read behind instead of in front for both and the contradiction evidence is blind. Check the
//! orientation of such scans (a shaded view shows an inverted scan dark).
use surfacecraft_geom::{Mesh, Point3, Transform};

/// Share of the shared edges of a scan that may be wound against each other (or shared by more than
/// two triangles) before its orientation is not trusted. Measured on the synthetic scans and on the
/// corpus (see the pull request): 0 for a welded mesh; a flipped or duplicated tenth reads 0.1 and up.
pub(super) const MAX_WINDING_DEFECT: f64 = 0.02;
/// Share of the matched points whose normals oppose the other scan's above which one scan is
/// inverted relative to the other (right poses read about 0, an inverted scan about 1).
pub(super) const MAX_OPPOSED: f64 = 0.5;

/// What the topology of a scan says about its orientation.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(super) struct Winding {
    /// Share of the edges shared by two or more triangles that are not traversed once in each
    /// direction by exactly two (0 for a consistently wound manifold).
    pub defect: f64,
    /// Signed volume of the mesh when it is closed (every edge shared by exactly two triangles), else `None`.
    pub volume: Option<f64>,
}

impl Winding {
    /// Why the orientation of this scan is not trusted, if it is not.
    pub(super) fn problem(&self) -> Option<&'static str> {
        if !(self.defect <= MAX_WINDING_DEFECT) {
            return Some("the winding of its triangles is not consistent");
        }
        match self.volume {
            Some(v) if v < 0.0 => Some("it is closed and turned inside out"),
            _ => None,
        }
    }
}

/// The winding of `mesh`; vertices are matched by position (like the borders), `diag` its scale.
pub(super) fn winding(mesh: &Mesh, diag: f64) -> Winding {
    let q = 1.0 / (1e-6 * diag).max(1e-300);
    let cell = |p: &Point3| [(p.x * q).round() as i64, (p.y * q).round() as i64, (p.z * q).round() as i64];
    // (smaller cell, larger cell, 1 when the triangle runs from the larger to the smaller)
    let mut half: Vec<([i64; 3], [i64; 3], u8)> = Vec::with_capacity(mesh.triangle_count() * 3);
    let mut volume = 0.0;
    for i in 0..mesh.triangle_count() {
        let Some(c) = mesh.triangle(i) else { continue };
        volume += c[0].coords.dot(&c[1].coords.cross(&c[2].coords)) / 6.0;
        for k in 0..3 {
            let (a, b) = (cell(&c[k]), cell(&c[(k + 1) % 3]));
            if a != b {
                half.push((a.min(b), a.max(b), u8::from(a > b)));
            }
        }
    }
    surfacecraft_geom::par::sort_unstable(&mut half);
    let (mut shared, mut bad, mut single) = (0usize, 0usize, 0usize);
    let mut i = 0;
    while i < half.len() {
        let mut j = i + 1;
        while j < half.len() && half.get(j).map(|h| (h.0, h.1)) == half.get(i).map(|h| (h.0, h.1)) {
            j += 1;
        }
        match j - i {
            1 => single += 1,
            2 => {
                shared += 1;
                bad += usize::from(half.get(i).map(|h| h.2) == half.get(i + 1).map(|h| h.2));
            }
            _ => {
                shared += 1;
                bad += 1;
            }
        }
        i = j;
    }
    let defect = if shared == 0 { 0.0 } else { bad as f64 / shared as f64 };
    let closed = single == 0 && shared > 0 && bad == 0;
    Winding { defect, volume: closed.then_some(volume) }
}

/// Share of the points of `pts` with unit normals `nrm` (moved by `t`) lying within `tau` of
/// `target` whose normal opposes the normal of the triangle of `target` they lie on. NaN when fewer
/// than `MIN_MATCHED` points matched (nothing to compare: the orientation is unknown).
pub(super) fn opposed_share(
    bvh: &surfacecraft_geom::Bvh,
    pts: &[Point3],
    nrm: &[surfacecraft_geom::Vec3],
    t: &Transform,
    tau: f64,
) -> f64 {
    const MIN_MATCHED: usize = 30;
    let votes: Vec<Option<bool>> = surfacecraft_geom::par::map(pts.len(), |i| {
        let (p, n) = (pts.get(i)?, nrm.get(i)?);
        let h = bvh.closest(&t.apply(p), tau)?;
        let other = bvh.triangle_normal(h.triangle)?;
        Some((t.rotation * n).dot(&other) < 0.0)
    });
    let matched = votes.iter().flatten().count();
    if matched < MIN_MATCHED {
        return f64::NAN;
    }
    votes.iter().flatten().filter(|v| **v).count() as f64 / matched as f64
}

#[cfg(test)]
mod tests {
    use super::*;
    use surfacecraft_geom::{Vec3, synth};

    fn flipped(m: &Mesh, every: usize) -> Mesh {
        let mut out = m.clone();
        for (i, t) in out.triangles.iter_mut().enumerate() {
            if i % every == 0 {
                t.swap(1, 2);
            }
        }
        out
    }

    #[test]
    fn a_welded_closed_mesh_is_consistent_and_a_flipped_one_is_inverted() {
        let ball = synth::sphere(Point3::origin(), 10.0, 32, 16);
        let w = winding(&ball, 20.0);
        assert!(w.defect <= 1e-12 && w.volume.is_some_and(|v| v > 0.0), "{w:?}");
        assert_eq!(w.problem(), None);
        let inside_out = flipped(&ball, 1);
        let w = winding(&inside_out, 20.0);
        assert!(w.defect <= 1e-12 && w.volume.is_some_and(|v| v < 0.0), "{w:?}");
        assert!(w.problem().is_some());
    }

    #[test]
    fn mixed_winding_and_duplicated_triangles_are_defects_of_a_tenth_and_more() {
        let plane = synth::plane(Point3::origin(), Vec3::x(), Vec3::y(), 40.0, 40.0, 30, 30);
        assert_eq!(winding(&plane, 60.0).problem(), None);
        let mixed = winding(&flipped(&plane, 5), 60.0);
        assert!(mixed.defect > 5.0 * MAX_WINDING_DEFECT, "{mixed:?}");
        let mut doubled = plane.clone();
        let extra: Vec<[u32; 3]> = plane.triangles.iter().step_by(5).copied().collect();
        doubled.triangles.extend(extra);
        let dup = winding(&doubled, 60.0);
        assert!(dup.defect > 5.0 * MAX_WINDING_DEFECT, "{dup:?}");
        assert!(dup.problem().is_some());
    }

    #[test]
    fn opposed_normals_are_counted_and_too_few_matches_are_unknown() {
        let plane = synth::plane(Point3::origin(), Vec3::x(), Vec3::y(), 40.0, 40.0, 20, 20);
        let bvh = surfacecraft_geom::Bvh::build(&plane);
        let pts: Vec<Point3> = (0..100).map(|i| Point3::new(i as f64 * 0.3, 10.0, 0.0)).collect();
        let up = vec![Vec3::z(); pts.len()];
        let down = vec![-Vec3::z(); pts.len()];
        let id = Transform::identity();
        let same = opposed_share(&bvh, &pts, &up, &id, 0.1);
        let opposite = opposed_share(&bvh, &pts, &down, &id, 0.1);
        assert!(same.abs() < 1e-12 && (opposite - 1.0).abs() < 1e-12, "{same} {opposite}");
        assert!(opposed_share(&bvh, &pts[..5], &up[..5], &id, 0.1).is_nan());
    }
}
