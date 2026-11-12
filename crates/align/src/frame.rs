//! The in-plane part of an automatic pose: once the base face fixes z, the side faces fix the
//! rotation about z (their normals vote on a 90-degree-periodic angle), the longer side of the
//! footprint becomes x, and the remaining sign ambiguity is settled by where the material is.
use crate::faces::{FaceSet, PlaneFace, plane_face};
use nalgebra::SymmetricEigen;
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use surfacecraft_geom::{Mat3, Point3, Transform, Vec3, any_perpendicular, normalize};

/// Side normals are within about 17 degrees of the base plane (|n·z| below this).
const SIDE_LIMIT: f64 = 0.3;
/// Refined side planes are closer to perpendicular to z than this (|n·z|).
const SIDE_PLANE_LIMIT: f64 = 0.1;
/// A side plane takes part in the refinement from this share of the total area.
const SIDE_PLANE_SHARE: f64 = 0.03;
/// Concentration of the 4-fold vote below which the sides do not define a direction.
const MIN_CONCENTRATION: f64 = 0.3;
/// Footprints whose spreads differ less than this (relative) have no principal direction.
const ISOTROPY: f64 = 0.05;
/// Extents are taken between these quantiles of the sampled vertices (spikes, debris).
const QUANTILE: f64 = 0.005;
/// A side counts as longer when its extent exceeds the other by this share.
const EXTENT_TIE: f64 = 0.01;
/// The height-weighted centroid sits off the base centre by at least this share of the side (a
/// part this symmetric has no preferred direction; the share only keeps rounding out).
const SIGN_EPS: f64 = 1e-4;
/// Vertices sampled for extents.
const SAMPLE_MAX: usize = 100_000;

/// How the rotation about the base normal was found.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub enum InPlane {
    /// From the side faces perpendicular to the base.
    SideFaces,
    /// From the principal directions of the footprint (the sides gave none).
    Footprint,
    /// Nothing fixes it (a turned part, a disc): the least rotation from the current pose.
    Free,
}

/// What `side_frame` needs of the mesh.
pub(crate) struct Context<'a> {
    pub set: &'a FaceSet<'a>,
    pub rims: &'a [PlaneFace],
    pub sample: &'a [Point3],
}

/// A stride sample of at most [`SAMPLE_MAX`] vertices.
pub(crate) fn sample_vertices(positions: &[Point3]) -> Vec<Point3> {
    let stride = positions.len().div_ceil(SAMPLE_MAX).max(1);
    positions.iter().step_by(stride).copied().filter(surfacecraft_geom::finite).collect()
}

/// The extent of the sample along `dir` between the 0.5 % and 99.5 % quantiles.
pub(crate) fn extent(sample: &[Point3], dir: &Vec3) -> (f64, f64) {
    let mut d: Vec<f64> = sample.iter().map(|p| p.coords.dot(dir)).collect();
    if d.is_empty() {
        return (0.0, 0.0);
    }
    d.sort_by(f64::total_cmp);
    let at = |q: f64| d.get((q * (d.len() - 1) as f64).round() as usize).copied().unwrap_or(0.0);
    (at(QUANTILE), at(1.0 - QUANTILE))
}

fn angle_of(r: &Mat3) -> f64 {
    Transform::from_parts(r, Vec3::zeros()).angle()
}

fn rows(x: &Vec3, y: &Vec3, z: &Vec3) -> Mat3 {
    Mat3::from_rows(&[x.transpose(), y.transpose(), z.transpose()])
}

/// Rotation (rows x, y, z) that puts `z` on world Z by the shortest arc.
pub(crate) fn least_rotation(z: &Vec3) -> Mat3 {
    let x_proj = Vec3::x() - z * z.x;
    let x = normalize(x_proj).or_else(|| normalize(Vec3::y() - z * z.y)).unwrap_or_else(Vec3::x);
    let y = z.cross(&x);
    rows(&x, &y, z)
}

/// Votes of normals on `e^{i4φ}` (φ measured in the plane (u, v)): (Σ w cos 4φ, Σ w sin 4φ, Σ w).
#[derive(Default, Clone, Copy)]
struct Votes(f64, f64, f64);

impl Votes {
    fn add(&mut self, n: &Vec3, u: &Vec3, v: &Vec3, w: f64) {
        let phi = n.dot(v).atan2(n.dot(u));
        self.0 += w * (4.0 * phi).cos();
        self.1 += w * (4.0 * phi).sin();
        self.2 += w;
    }
    fn angle(self) -> f64 {
        self.1.atan2(self.0) / 4.0
    }
    fn concentration(self) -> f64 {
        if self.2 > 0.0 { self.0.hypot(self.1) / self.2 } else { 0.0 }
    }
}

fn coarse_votes(inp: &Context, z: &Vec3, u: &Vec3, v: &Vec3) -> Votes {
    let mut votes = Votes::default();
    for f in inp.set.faces.iter().filter(|f| f.n.dot(z).abs() < SIDE_LIMIT) {
        votes.add(&f.n, u, v, f.w);
    }
    for r in inp.rims.iter().filter(|r| r.normal.dot(z).abs() < SIDE_LIMIT) {
        votes.add(&r.normal, u, v, r.area);
    }
    votes
}

/// The coarse angle replaced by the vote of the fitted side planes (exact to a hundredth of a
/// degree where the faces are large).
fn refine_angle(inp: &Context, z: &Vec3, u: &Vec3, v: &Vec3, theta: f64) -> f64 {
    let mut votes = Votes::default();
    for k in 0..4 {
        let a = theta + k as f64 * std::f64::consts::FRAC_PI_2;
        let seed = u * a.cos() + v * a.sin();
        let Some(p) = plane_face(inp.set, &seed) else { continue };
        if p.area >= SIDE_PLANE_SHARE * inp.set.total && p.normal.dot(z).abs() < SIDE_PLANE_LIMIT {
            votes.add(&p.normal, u, v, p.area);
        }
    }
    for r in inp.rims.iter().filter(|r| r.normal.dot(z).abs() < SIDE_PLANE_LIMIT) {
        votes.add(&r.normal, u, v, r.area);
    }
    if votes.2 > 0.0 { votes.angle() } else { theta }
}

/// The principal direction of the footprint (area-weighted triangle centroids in the plane), or
/// `None` when it is isotropic.
fn footprint_angle(set: &FaceSet, u: &Vec3, v: &Vec3) -> Option<f64> {
    let (mut suu, mut svv, mut suv) = (0.0, 0.0, 0.0);
    let c = set.centroid.coords;
    for f in &set.faces {
        let d = f.c.coords - c;
        let (a, b) = (d.dot(u), d.dot(v));
        suu += f.w * a * a;
        svv += f.w * b * b;
        suv += f.w * a * b;
    }
    let m = nalgebra::Matrix2::new(suu, suv, suv, svv);
    let eig = SymmetricEigen::new(m);
    let (hi, lo) = (eig.eigenvalues.max(), eig.eigenvalues.min());
    if !(hi > 0.0) || (hi - lo) / (hi + lo) < ISOTROPY {
        return None;
    }
    let col = if eig.eigenvalues[0] >= eig.eigenvalues[1] { 0 } else { 1 };
    Some(eig.eigenvectors[(1, col)].atan2(eig.eigenvectors[(0, col)]))
}

/// The two unit in-plane axes (x, y) for base normal `z`, how they were found, and the
/// 90-degree-periodic angle's concentration.
fn in_plane_axes(inp: &Context, z: &Vec3) -> (Vec3, InPlane) {
    let u = any_perpendicular(z);
    let v = z.cross(&u);
    let votes = coarse_votes(inp, z, &u, &v);
    let (theta, how) = if votes.concentration() >= MIN_CONCENTRATION {
        (Some(refine_angle(inp, z, &u, &v, votes.angle())), InPlane::SideFaces)
    } else {
        (footprint_angle(inp.set, &u, &v), InPlane::Footprint)
    };
    match theta {
        Some(t) => (u * t.cos() + v * t.sin(), how),
        None => (least_rotation(z).row(0).transpose(), InPlane::Free),
    }
}

/// Which way the material leans along `dir`: the sign of the offset of the centroid of the
/// surface, weighted by height above the base, from the base centre; 0 within the tolerance.
/// (Tall features count most: a boss on one side decides, the base face and flat tops do not.)
fn material_side_of(inp: &Context, z: &Vec3, origin: &Point3, dir: &Vec3) -> i8 {
    let (mut moment, mut weight) = (0.0, 0.0);
    for f in &inp.set.faces {
        let h = (f.c - origin).dot(z).max(0.0) * f.w;
        moment += h * (f.c - origin).dot(dir);
        weight += h;
    }
    let (lo, hi) = extent(inp.sample, dir);
    if !(weight > 0.0) || (moment / weight).abs() <= SIGN_EPS * (hi - lo) {
        0
    } else if moment > 0.0 {
        1
    } else {
        -1
    }
}

/// The rotation (rows x, y, z) for base normal `z` (pointing into the material).
///
/// x is the longer side of the footprint (the sides' direction, or the footprint's principal
/// direction), y = z × x. Of the two directions of x the one with more material at +x is taken
/// (see `material_side_of`); when x is a tie, the same test on y; when that is a tie too, the
/// one needing the smaller rotation from the current pose. `origin` is the base centre.
pub(crate) fn side_frame(inp: &Context, z: &Vec3, origin: &Point3) -> (Mat3, InPlane) {
    let (mut x, how) = in_plane_axes(inp, z);
    let mut y = z.cross(&x);
    if how == InPlane::Free {
        return (rows(&x, &y, z), how);
    }
    let (ex, ey) = (extent(inp.sample, &x), extent(inp.sample, &y));
    if (ey.1 - ey.0) > (ex.1 - ex.0) * (1.0 + EXTENT_TIE) {
        (x, y) = (y, -x);
    }
    let side = match material_side_of(inp, z, origin, &x) {
        0 => material_side_of(inp, z, origin, &y),
        s => s,
    };
    let keep = rows(&x, &y, z);
    let flip = rows(&-x, &-y, z);
    let pick_flip = match side {
        1 => false,
        -1 => true,
        _ => angle_of(&flip) < angle_of(&keep),
    };
    (if pick_flip { flip } else { keep }, how)
}

/// Rotation that levels all near-axis-aligned faces by area: the weighted least-squares
/// compromise (Kabsch of the normals onto the nearest frame axes), applied after `r`.
pub(crate) fn level_all_faces(set: &FaceSet, r: &Mat3, near_cos: f64) -> Mat3 {
    let mut h = Mat3::zeros();
    for f in &set.faces {
        let m = r * f.n;
        let k = (0..3).max_by(|&i, &j| m[i].abs().total_cmp(&m[j].abs())).unwrap_or(0);
        if m[k].abs() < near_cos {
            continue;
        }
        let mut axis = Vec3::zeros();
        axis[k] = m[k].signum();
        h += (axis * m.transpose()) * f.w;
    }
    let svd = h.svd(true, true);
    let (Some(u), Some(vt)) = (svd.u, svd.v_t) else { return *r };
    let mut d = Mat3::identity();
    d[(2, 2)] = (u * vt).determinant().signum();
    let q = u * d * vt;
    q * r
}
