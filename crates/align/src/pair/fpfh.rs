//! Fast point feature histograms (Rusu, Blodow, Beetz, ICRA 2009), written from the paper.
//!
//! A descriptor summarises how the normals around a point turn: three angles of the Darboux frame
//! of every pair of neighbours, binned into 3 × 11 bins, blended with the histograms of the
//! neighbours. It describes the local shape and does not depend on the pose, so descriptors of
//! the same surface in two scans match even when the scans overlap only partly.
use super::sample::Samples;
use surfacecraft_geom::{Point3, Vec3};

/// Bins per angle.
const BINS: usize = 11;
/// Descriptor length.
pub(super) const DIM: usize = 3 * BINS;
/// Fewer neighbours than this and a point has no usable descriptor.
const MIN_NEIGHBOURS: usize = 8;

/// One descriptor.
pub(super) type Desc = [f32; DIM];

fn bin(v: f64, lo: f64, hi: f64) -> usize {
    let t = ((v - lo) / (hi - lo)).clamp(0.0, 1.0 - 1e-9);
    (t * BINS as f64) as usize
}

/// The three angles of the pair (`p1`, `n1`), (`p2`, `n2`): the Darboux frame is built on the
/// point whose normal is closer to the line joining them.
fn pair_angles(p1: &Point3, n1: &Vec3, p2: &Point3, n2: &Vec3) -> Option<[f64; 3]> {
    let d = p2 - p1;
    let dist = d.norm();
    if !(dist > 1e-12) {
        return None;
    }
    let c1 = n1.dot(&d) / dist;
    let c2 = n2.dot(&d) / dist;
    let (u, nt, line, f3) = if c1.abs() >= c2.abs() { (*n1, *n2, d / dist, c1) } else { (*n2, *n1, -d / dist, -c2) };
    let v = line.cross(&u);
    let Some(v) = surfacecraft_geom::normalize(v) else { return Some([0.0, 0.0, f3]) };
    let w = u.cross(&v);
    Some([w.dot(&nt).atan2(u.dot(&nt)), v.dot(&nt), f3])
}

/// Normalises each of the three sub-histograms to sum 1.
fn normalise(h: &mut [f32; DIM]) {
    for part in h.chunks_mut(BINS) {
        let s: f32 = part.iter().sum();
        if s > 0.0 {
            part.iter_mut().for_each(|x| *x /= s);
        }
    }
}

/// Simple histogram of the pair angles between `i` and its neighbours.
fn spfh(s: &Samples, i: usize, nbrs: &[usize]) -> Desc {
    let mut h = [0.0f32; DIM];
    let (Some(p), Some(n)) = (s.pts.get(i), s.nrm.get(i)) else { return h };
    for &j in nbrs {
        let (Some(q), Some(m)) = (s.pts.get(j), s.nrm.get(j)) else { continue };
        let Some([a, b, c]) = pair_angles(p, n, q, m) else { continue };
        for (k, (v, lo, hi)) in
            [(a, -std::f64::consts::PI, std::f64::consts::PI), (b, -1.0, 1.0), (c, -1.0, 1.0)].into_iter().enumerate()
        {
            if let Some(x) = h.get_mut(k * BINS + bin(v, lo, hi)) {
                *x += 1.0;
            }
        }
    }
    normalise(&mut h);
    h
}

/// A descriptor per sample, `None` where the point has too few neighbours within `radius`.
pub(super) fn describe(s: &Samples, radius: f64) -> Vec<Option<Desc>> {
    let n = s.pts.len();
    let nbrs: Vec<Vec<usize>> = surfacecraft_geom::par::map(n, |i| {
        let Some(p) = s.pts.get(i) else { return Vec::new() };
        s.tree.within(p, radius).into_iter().filter(|&j| j != i).collect()
    });
    let simple: Vec<Desc> = surfacecraft_geom::par::map(n, |i| spfh(s, i, nbrs.get(i).map_or(&[][..], |v| v)));
    surfacecraft_geom::par::map(n, |i| {
        let list = nbrs.get(i)?;
        if list.len() < MIN_NEIGHBOURS {
            return None;
        }
        let mut acc = *simple.get(i)?;
        let p = s.pts.get(i)?;
        for &j in list {
            let (Some(q), Some(sj)) = (s.pts.get(j), simple.get(j)) else { continue };
            let w = (1.0 / (q - p).norm().max(1e-9)) as f32;
            acc.iter_mut().zip(sj).for_each(|(a, x)| *a += w * x);
        }
        normalise(&mut acc);
        Some(acc)
    })
}

#[cfg(test)]
mod tests {
    use super::super::sample::sample;
    use super::*;
    use surfacecraft_geom::{Cancel, Transform, synth};

    fn dist2(a: &Desc, b: &Desc) -> f32 {
        a.iter().zip(b).map(|(x, y)| (x - y) * (x - y)).sum()
    }

    /// The descriptor of a point does not change when the whole surface is moved.
    #[test]
    fn descriptors_do_not_depend_on_the_pose() {
        let m = synth::bracket();
        let moved = m.transformed(
            &Transform::rotation_axis_angle(&Vec3::new(0.3, 1.0, 0.2), 1.9)
                .then_after(&Transform::translation(Vec3::new(40.0, 5.0, -9.0))),
        );
        let a = sample(&m, 1500, 2.5, 0, &Cancel::new()).unwrap();
        let b = sample(&moved, 1500, 2.5, 0, &Cancel::new()).unwrap();
        let (da, db) = (describe(&a, 12.0), describe(&b, 12.0));
        // same seed, same triangles: sample i of `b` is the image of sample i of `a`
        let mut worst = 0.0f32;
        for (x, y) in da.iter().zip(&db) {
            if let (Some(x), Some(y)) = (x, y) {
                worst = worst.max(dist2(x, y));
            }
        }
        assert!(worst < 1e-3, "descriptor changed by {worst}");
    }
}
