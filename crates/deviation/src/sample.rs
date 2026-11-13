//! Samples: scan points measured against a reference, with their signed deviations and the reference's normals.

use super::stats::*;
use super::*;

/// One scan point measured against a reference: its signed deviation and whether it faces it.
///
/// Sign convention: the deviation is positive when the scan lies outside the reference material,
/// judged by the reference surface normal (outward for bodies, spheres, cylinders, cones and tori;
/// for a plane, see [`oriented_plane`]). A point whose own normal points away from the reference
/// normal (dot < 0) is not facing: its nearest reference surface is then another wall of the part
/// (the far side of a thin plate), so it is left out of the statistics.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Sample {
    /// Signed deviation (mm); NaN when the point is not measured.
    pub d: f64,
    /// False when the scan normal points away from the reference normal.
    pub facing: bool,
    /// Unit reference normal at the nearest reference point (zero when unknown, as for freeform
    /// surfaces). Areas are projected along it, see [`hotspots`].
    pub reference: [f64; 3],
}

impl Sample {
    /// A point that was not measured (outside the region, or farther than the reach).
    pub const UNMEASURED: Sample = Sample { d: f64::NAN, facing: false, reference: [0.0; 3] };

    /// Measured and facing: counted in the statistics.
    #[must_use]
    pub fn counted(&self) -> bool {
        self.d.is_finite() && self.facing
    }

    /// Measured, but not facing the reference: left out of the statistics.
    #[must_use]
    pub fn not_facing(&self) -> bool {
        self.d.is_finite() && !self.facing
    }
}

/// Whether a scan normal faces the reference normal. A missing or non-finite comparison counts as
/// facing: nothing in the data contradicts the measurement.
fn faces(scan: Option<&Vec3>, reference: &Vec3) -> bool {
    scan.is_none_or(|n| !(n.dot(reference) < 0.0))
}

/// Samples of points against a primitive. `normals[i]` is the scan normal of `points[i]` (missing
/// normals count as facing).
#[must_use]
pub fn samples_primitive(points: &[Point3], normals: &[Vec3], prim: &Primitive) -> Vec<Sample> {
    points
        .iter()
        .enumerate()
        .map(|(i, p)| {
            let n = prim.normal(p);
            Sample { d: prim.distance(p), facing: faces(normals.get(i), &n), reference: [n.x, n.y, n.z] }
        })
        .collect()
}

/// Samples of points against a mesh surface: the sign comes from the normal of the nearest
/// triangle, and points farther than `max_distance` are not measured. `normals[i]` is the scan
/// normal of `points[i]`.
#[must_use]
pub fn samples_mesh(points: &[Point3], normals: &[Vec3], target: &Bvh, max_distance: f64) -> Vec<Sample> {
    samples_mesh_tagged(points, normals, target, max_distance).into_iter().map(|(s, _)| s).collect()
}

/// [`samples_mesh`] with the index of the nearest reference triangle of each point (`None` when it
/// is not measured), so that points can be attributed to the faces of the reference.
#[must_use]
pub fn samples_mesh_tagged(
    points: &[Point3],
    normals: &[Vec3],
    target: &Bvh,
    max_distance: f64,
) -> Vec<(Sample, Option<usize>)> {
    surfacecraft_geom::par::map(points.len(), |i| {
        let Some(p) = points.get(i) else { return (Sample::UNMEASURED, None) };
        let Some(h) = target.closest(p, max_distance) else { return (Sample::UNMEASURED, None) };
        (mesh_sample(p, normals.get(i), target, &h), Some(h.triangle))
    })
}

/// The sample of point `p` against its nearest hit `h` on `target`.
fn mesh_sample(p: &Point3, scan_normal: Option<&Vec3>, target: &Bvh, h: &surfacecraft_geom::SurfaceHit) -> Sample {
    let Some(n) = target.triangle_normal(h.triangle) else {
        // a degenerate reference triangle has no side: its distance is taken as outside
        return Sample { d: h.distance, facing: true, reference: [0.0; 3] };
    };
    let outside = (p - h.point).dot(&n) >= 0.0;
    Sample {
        d: if outside { h.distance } else { -h.distance },
        facing: faces(scan_normal, &n),
        reference: [n.x, n.y, n.z],
    }
}

/// The deviation values of the counted points, NaN for the others (for the colour map).
#[must_use]
pub fn counted_values(samples: &[Sample]) -> Vec<f64> {
    samples.iter().map(|s| if s.counted() { s.d } else { f64::NAN }).collect()
}

/// [`stats`] of the samples, with the points that do not face the reference counted apart.
#[must_use]
pub fn stats_of(samples: &[Sample], tolerance: f64) -> DeviationStats {
    let mut st = stats(&counted_values(samples), tolerance);
    st.not_facing = samples.iter().filter(|s| s.not_facing()).count();
    st
}

/// The primitive with a plane's normal turned to agree with `normals` (the scan normals of the
/// points it describes). A plane has no inherent side, so its material side comes from the scan:
/// the normal is flipped when the scan normals point against it on balance. Other primitives are
/// returned unchanged, since their normals point outward by construction.
#[must_use]
pub fn oriented_plane(prim: &Primitive, normals: &[Vec3]) -> Primitive {
    if let Primitive::Plane { point, normal } = prim {
        let n = Vec3::new(normal[0], normal[1], normal[2]);
        let vote: f64 =
            normals.iter().filter(|v| surfacecraft_geom::finite_vec(v) && v.norm() > 0.5).map(|v| v.dot(&n)).sum();
        if vote < 0.0 {
            return Primitive::Plane { point: *point, normal: [-normal[0], -normal[1], -normal[2]] };
        }
    }
    prim.clone()
}

/// Unused helper kept for symmetry with primitives: the outward normal of a primitive at `p`.
#[must_use]
pub fn primitive_normal(prim: &Primitive, p: &Point3) -> Vec3 {
    prim.normal(p)
}
