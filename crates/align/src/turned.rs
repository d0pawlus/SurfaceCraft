//! Turned parts (rotational symmetry): the face normals of a body of revolution concentrate on a
//! great circle (the side) plus two poles (the end faces). Detection finds the axis from the
//! circle; the end faces' planes then give it exactly (see `auto`).
use crate::faces::FaceSet;
use crate::frame::extent;
use nalgebra::SymmetricEigen;
use surfacecraft_geom::{Mat3, Point3, Vec3, any_perpendicular, normalize};

/// Normals this close to the equator (|n·a| below the sine of 11.5 degrees) are side normals.
const BAND: f64 = 0.2;
/// The band used to refine the axis (wide enough that noisy side normals stay in).
const REFINE_BAND: f64 = 0.3;
/// A turned part has at least this share of its area on the side.
const MIN_SIDE_SHARE: f64 = 0.4;
/// Azimuth bins around the axis.
const BINS: usize = 24;
/// The side normals cover at least this many azimuth bins ...
const MIN_OCCUPIED: usize = 20;
/// ... where "covered" is this share of the mean bin weight ...
const OCCUPIED_SHARE: f64 = 0.3;
/// ... and the four heaviest bins hold at most this share (a box has 4 bins with everything).
const MAX_TOP_FOUR: f64 = 0.45;
const REFINE_ROUNDS: usize = 3;
/// Sectors around the axis in which the outer radius is compared.
const SECTORS: usize = 12;
/// The outer radius differs between sectors by at most this share of its mean (an ellipse of
/// radii 12 and 10 differs by 18 %; a keyway narrower than a sector changes nothing).
const MAX_RADIUS_SPREAD: f64 = 0.03;
/// Every sector holds at least this share of the sampled vertices.
const MIN_SECTOR_SHARE: f64 = 0.005;
/// The outer radius of a sector: this quantile of its vertices' distances from the axis. A scan's
/// floating debris patch on the side (a few vertices a few spacings out) holds more than 1 % of a
/// sector's vertices and once moved one sector's 0.99 quantile out by 2 mm, so a pin was not
/// turned; the side of a turned part holds far more than 5 % of every sector's outer vertices.
const OUTER_QUANTILE: f64 = 0.95;

/// The axis of a turned part.
pub(crate) struct Turned {
    /// Unit axis (the sign is arbitrary).
    pub axis: Vec3,
}

/// Is the part rotationally symmetric about some axis, with its area mostly on the side?
///
/// The normals only nominate an axis (an elliptical extrusion has normals all around too); the
/// vertices then confirm the rotational symmetry (see [`is_revolved`]).
pub(crate) fn detect(set: &FaceSet, sample: &[Point3]) -> Option<Turned> {
    let mut cov = Mat3::zeros();
    for f in &set.faces {
        cov += (f.n * f.n.transpose()) * f.w;
    }
    let eig = SymmetricEigen::new(cov);
    let mut best: Option<(f64, Vec3)> = None;
    for k in 0..3 {
        let Some(axis) = normalize(eig.eigenvectors.column(k).into_owned()) else { continue };
        let Some(share) = side_share(set, &axis) else { continue };
        if best.as_ref().is_none_or(|b| share > b.0) {
            best = Some((share, axis));
        }
    }
    let (_, seed) = best?;
    let axis = refine(set, seed);
    is_revolved(sample, &axis).then_some(Turned { axis })
}

/// Is the outer radius about `axis` the same in every sector of the circumference? The centre is
/// the middle of the sample's extent across the axis. A flat wider than a sector (or deeper than
/// 3 % of the radius) breaks it, a narrow keyway does not.
pub(crate) fn is_revolved(sample: &[Point3], axis: &Vec3) -> bool {
    let u = any_perpendicular(axis);
    let v = axis.cross(&u);
    let (u0, u1) = extent(sample, &u);
    let (v0, v1) = extent(sample, &v);
    let (cu, cv) = (0.5 * (u0 + u1), 0.5 * (v0 + v1));
    let mut radii: Vec<Vec<f64>> = vec![Vec::new(); SECTORS];
    for p in sample {
        let (a, b) = (p.coords.dot(&u) - cu, p.coords.dot(&v) - cv);
        let phi = b.atan2(a) + std::f64::consts::PI;
        let k = ((phi / std::f64::consts::TAU * SECTORS as f64) as usize).min(SECTORS - 1);
        if let Some(r) = radii.get_mut(k) {
            r.push(a.hypot(b));
        }
    }
    let outer: Vec<f64> = radii
        .iter_mut()
        .filter(|r| !r.is_empty() && r.len() as f64 >= MIN_SECTOR_SHARE * sample.len() as f64)
        .filter_map(|r| {
            r.sort_by(f64::total_cmp);
            r.get(((r.len() - 1) as f64 * OUTER_QUANTILE).round() as usize).copied()
        })
        .collect();
    let mean = outer.iter().sum::<f64>() / outer.len().max(1) as f64;
    let (lo, hi) = outer.iter().fold((f64::INFINITY, f64::NEG_INFINITY), |(l, h), &r| (l.min(r), h.max(r)));
    outer.len() == SECTORS && mean > 0.0 && (hi - lo) <= MAX_RADIUS_SPREAD * mean
}

/// The share of the area on the side of a body of revolution about `axis`, or `None` when the
/// side normals are not spread evenly around it.
fn side_share(set: &FaceSet, axis: &Vec3) -> Option<f64> {
    let u = any_perpendicular(axis);
    let v = axis.cross(&u);
    let mut bins = [0.0_f64; BINS];
    let mut band = 0.0;
    for f in set.faces.iter().filter(|f| f.n.dot(axis).abs() < BAND) {
        let phi = f.n.dot(&v).atan2(f.n.dot(&u));
        let i = (((phi + std::f64::consts::PI) / std::f64::consts::TAU * BINS as f64) as usize).min(BINS - 1);
        bins[i] += f.w;
        band += f.w;
    }
    let share = band / set.total;
    if share < MIN_SIDE_SHARE {
        return None;
    }
    let occupied = bins.iter().filter(|&&w| w >= OCCUPIED_SHARE * band / BINS as f64).count();
    let mut sorted = bins;
    sorted.sort_by(|a, b| b.total_cmp(a));
    let top_four: f64 = sorted.iter().take(4).sum();
    (occupied >= MIN_OCCUPIED && top_four <= MAX_TOP_FOUR * band).then_some(share)
}

/// The axis as the direction least spanned by the side normals.
fn refine(set: &FaceSet, seed: Vec3) -> Vec3 {
    let mut axis = seed;
    for _ in 0..REFINE_ROUNDS {
        let mut cov = Mat3::zeros();
        for f in set.faces.iter().filter(|f| f.n.dot(&axis).abs() < REFINE_BAND) {
            cov += (f.n * f.n.transpose()) * f.w;
        }
        let eig = SymmetricEigen::new(cov);
        let lowest = (0..3).min_by(|&i, &j| eig.eigenvalues[i].total_cmp(&eig.eigenvalues[j])).unwrap_or(0);
        let Some(mut next) = normalize(eig.eigenvectors.column(lowest).into_owned()) else { break };
        if next.dot(&seed) < 0.0 {
            next = -next;
        }
        axis = next;
    }
    axis
}

#[cfg(test)]
mod tests {
    use super::*;
    use surfacecraft_geom::{Point3, Rng, Transform, synth};

    fn set_of(m: &surfacecraft_geom::Mesh) -> FaceSet<'_> {
        FaceSet::new(m).unwrap()
    }

    fn sample_of(m: &surfacecraft_geom::Mesh) -> Vec<Point3> {
        crate::frame::sample_vertices(&m.positions)
    }

    #[test]
    fn a_pin_is_turned_about_its_axis() {
        let m = synth::cylinder(Point3::origin(), Vec3::z(), 12.0, 30.0, 120, 40);
        let t = Transform::rotation_axis_angle(&Vec3::new(1.0, 0.4, -0.2), 0.9);
        let mut moved = m.transformed(&t);
        synth::add_noise(&mut moved, 0.03, &mut Rng::new(1));
        let turned = detect(&set_of(&moved), &sample_of(&moved)).expect("a cylinder is turned");
        assert!(turned.axis.dot(&t.apply_vec(&Vec3::z())).abs() > 0.999, "{:?}", turned.axis);
    }

    #[test]
    fn boxes_and_blobs_are_not_turned() {
        let b = synth::cuboid(Point3::origin(), Vec3::new(40.0, 30.0, 15.0), 10);
        assert!(detect(&set_of(&b), &sample_of(&b)).is_none());
        let s = synth::sphere(Point3::origin(), 20.0, 48, 24);
        assert!(detect(&set_of(&s), &sample_of(&s)).is_none(), "a sphere's side is only a tenth of its area");
    }

    /// Points on the side of a cylinder of radius 12 about z, `n` around by `m` along.
    fn side(n: usize, m: usize) -> Vec<Point3> {
        (0..n)
            .flat_map(|i| {
                let a = std::f64::consts::TAU * i as f64 / n as f64;
                (0..m).map(move |j| Point3::new(12.0 * a.cos(), 12.0 * a.sin(), 30.0 * j as f64 / m as f64))
            })
            .collect()
    }

    /// A flat 1 mm deep (8 % of the radius) across about a sector and a half is no turned part,
    /// with the outer radius taken at the 0.95 quantile of each sector.
    #[test]
    fn a_shaft_with_a_flat_is_not_revolved() {
        let pts: Vec<Point3> =
            side(240, 30).into_iter().map(|p| if p.x > 11.0 { Point3::new(11.0, p.y, p.z) } else { p }).collect();
        assert!(is_revolved(&side(240, 30), &Vec3::z()));
        assert!(!is_revolved(&pts, &Vec3::z()));
    }

    /// A boss 2 mm proud holding 8 % of one sector's points moves that sector's outer radius: no
    /// turned part. A debris patch of 2 % of a sector does not.
    #[test]
    fn a_boss_on_one_sector_breaks_the_symmetry_and_debris_does_not() {
        let base = side(240, 30);
        // the sectors as `is_revolved` cuts them (its frame about the axis; the sample is centred)
        let (u, v) = (any_perpendicular(&Vec3::z()), Vec3::z().cross(&any_perpendicular(&Vec3::z())));
        let phi = |p: &Point3| p.coords.dot(&v).atan2(p.coords.dot(&u)) + std::f64::consts::PI;
        let width = std::f64::consts::TAU / SECTORS as f64;
        let in_sector = |p: &Point3| ((phi(p) / width) as usize).min(SECTORS - 1) == 3;
        let mid = |p: &Point3| (phi(p) - 3.5 * width).abs();
        let mut sector: Vec<usize> = (0..base.len()).filter(|&k| in_sector(&base[k])).collect();
        sector.sort_by(|&a, &b| mid(&base[a]).total_cmp(&mid(&base[b])));
        let proud = |share: f64| {
            let take = (sector.len() as f64 * share).ceil() as usize;
            let mut pts = base.clone();
            for &k in sector.iter().take(take) {
                let p = pts[k];
                let r = p.x.hypot(p.y);
                pts[k] = Point3::new(p.x * (r + 2.0) / r, p.y * (r + 2.0) / r, p.z);
            }
            pts
        };
        assert!(!is_revolved(&proud(0.08), &Vec3::z()));
        assert!(is_revolved(&proud(0.02), &Vec3::z()));
    }
}
