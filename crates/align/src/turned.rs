//! Turned parts (rotational symmetry): the face normals of a body of revolution concentrate on a
//! great circle (the side) plus two poles (the end faces). Detection finds the axis from the
//! circle; the end faces' planes then give it exactly (see `auto`).
use crate::faces::FaceSet;
use nalgebra::SymmetricEigen;
use surfacecraft_geom::{Mat3, Vec3, any_perpendicular, normalize};

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

/// The axis of a turned part.
pub(crate) struct Turned {
    /// Unit axis (the sign is arbitrary).
    pub axis: Vec3,
}

/// Is the part rotationally symmetric about some axis, with its area mostly on the side?
pub(crate) fn detect(set: &FaceSet) -> Option<Turned> {
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
    Some(Turned { axis: refine(set, seed) })
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

    fn set_of(m: &surfacecraft_geom::Mesh) -> FaceSet {
        FaceSet::new(m).unwrap()
    }

    #[test]
    fn a_pin_is_turned_about_its_axis() {
        let mut m = synth::cylinder(Point3::origin(), Vec3::z(), 12.0, 30.0, 120, 40);
        m.append(&synth::plane(Point3::new(-12.0, -12.0, 0.0), Vec3::x(), Vec3::y(), 24.0, 24.0, 12, 12));
        let t = Transform::rotation_axis_angle(&Vec3::new(1.0, 0.4, -0.2), 0.9);
        let mut moved = m.transformed(&t);
        synth::add_noise(&mut moved, 0.03, &mut Rng::new(1));
        let turned = detect(&set_of(&moved)).expect("a cylinder is turned");
        assert!(turned.axis.dot(&t.apply_vec(&Vec3::z())).abs() > 0.999, "{:?}", turned.axis);
    }

    #[test]
    fn boxes_and_blobs_are_not_turned() {
        let b = synth::cuboid(Point3::origin(), Vec3::new(40.0, 30.0, 15.0), 10);
        assert!(detect(&set_of(&b)).is_none());
        let s = synth::sphere(Point3::origin(), 20.0, 48, 24);
        assert!(detect(&set_of(&s)).is_none(), "a sphere's side is only a tenth of its area");
    }
}
