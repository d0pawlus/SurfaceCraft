//! Flat triangulation of a boundary loop: ear clipping in the loop's mean plane.
//!
//! The ear of every vertex is judged once and the judgement is kept: an ear stays valid when
//! another vertex is clipped (only points inside its triangle could spoil it, and fewer points
//! remain), a spoiled ear remembers the point that spoils it and is judged again only when that
//! point is clipped or its own neighbours change, and the best ear (largest area over the square of
//! the longest edge, the first on a tie) is the same as when every ear is judged afresh each round.
//! That takes a loop of n edges about n² steps instead of n³.
use crate::{Mesh, Point3, Vec3};

/// An ear whose double area is not above this is not convex enough to clip.
const MIN_EAR_CROSS: f64 = 1e-15;

/// Newell normal frame of a loop: centroid, two in-plane axes and the normal.
pub(crate) fn loop_frame(mesh: &Mesh, lp: &[u32]) -> (Point3, Vec3, Vec3, Vec3) {
    let mut n = Vec3::zeros();
    let mut c = Vec3::zeros();
    for (k, &a) in lp.iter().enumerate() {
        let p = mesh.positions[a as usize];
        let q = mesh.positions[lp[(k + 1) % lp.len()] as usize];
        n += Vec3::new((p.y - q.y) * (p.z + q.z), (p.z - q.z) * (p.x + q.x), (p.x - q.x) * (p.y + q.y));
        c += p.coords;
    }
    let n = crate::normalize(n).unwrap_or_else(Vec3::z);
    let x = crate::any_perpendicular(&n);
    let y = n.cross(&x);
    (Point3::from(c / lp.len() as f64), x, y, n)
}

/// What is known of the ear at one vertex of the remaining polygon.
#[derive(Clone, Copy)]
enum Ear {
    /// Not convex: judged again only when a neighbour changes.
    Flat,
    /// Another vertex lies inside the triangle.
    Blocked(usize),
    /// Clippable, with this quality.
    Valid(f64),
}

fn cross(a: [f64; 2], b: [f64; 2], c: [f64; 2]) -> f64 {
    (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
}

fn len2(u: [f64; 2], v: [f64; 2]) -> f64 {
    (u[0] - v[0]).powi(2) + (u[1] - v[1]).powi(2)
}

/// The ear at position `k` of `idx` (the remaining vertices, in loop order).
fn judge(p2: &[[f64; 2]], idx: &[usize], k: usize, sign: f64) -> Ear {
    let m = idx.len();
    let (ia, ib, ic) = (idx[(k + m - 1) % m], idx[k], idx[(k + 1) % m]);
    let (a, b, c) = (p2[ia], p2[ib], p2[ic]);
    let cr = cross(a, b, c) * sign;
    if cr <= MIN_EAR_CROSS {
        return Ear::Flat;
    }
    let blocker = idx.iter().copied().find(|&j| {
        if j == ia || j == ib || j == ic {
            return false;
        }
        let p = p2[j];
        cross(a, b, p) * sign > 0.0 && cross(b, c, p) * sign > 0.0 && cross(c, a, p) * sign > 0.0
    });
    match blocker {
        Some(j) => Ear::Blocked(j),
        None => Ear::Valid(cr / len2(a, b).max(len2(b, c)).max(len2(c, a)).max(1e-300)),
    }
}

/// Ear clipping in the loop's mean plane; false when the projected polygon is not simple enough.
#[cfg(test)]
pub(crate) fn ear_clip(mesh: &mut Mesh, lp: &[u32]) -> bool {
    ear_clip_avoiding(mesh, lp, &|_, _| false)
}

/// [`ear_clip`] that never clips an ear whose new diagonal `(a, b)` (mesh vertices) is one for
/// which `taken` says an edge exists already: a second pair of triangles on it would make it
/// non-manifold. False also when only such ears are left.
pub(crate) fn ear_clip_avoiding(mesh: &mut Mesh, lp: &[u32], taken: &dyn Fn(u32, u32) -> bool) -> bool {
    let (o, x, y, _) = loop_frame(mesh, lp);
    let p2: Vec<[f64; 2]> = lp
        .iter()
        .map(|&v| {
            let d = mesh.positions[v as usize] - o;
            [d.dot(&x), d.dot(&y)]
        })
        .collect();
    let area2: f64 = (0..p2.len())
        .map(|k| {
            let (a, b) = (p2[k], p2[(k + 1) % p2.len()]);
            a[0] * b[1] - b[0] * a[1]
        })
        .sum();
    let sign = if area2 >= 0.0 { 1.0 } else { -1.0 };
    let mut idx: Vec<usize> = (0..lp.len()).collect();
    let judge_ear = |idx: &[usize], k: usize| {
        let m = idx.len();
        let ear = judge(&p2, idx, k, sign);
        let (ia, ic) = (idx[(k + m - 1) % m], idx[(k + 1) % m]);
        match ear {
            Ear::Valid(_) if taken(lp[ia], lp[ic]) => Ear::Flat,
            other => other,
        }
    };
    let mut ears: Vec<Ear> = (0..lp.len()).map(|k| judge_ear(&idx, k)).collect();
    let mut out: Vec<[u32; 3]> = Vec::with_capacity(lp.len());
    while idx.len() > 3 {
        let m = idx.len();
        let mut best: Option<(usize, f64)> = None;
        for (k, &v) in idx.iter().enumerate() {
            if let Ear::Valid(q) = ears[v]
                && best.is_none_or(|(_, bq)| q > bq)
            {
                best = Some((k, q));
            }
        }
        let Some((k, _)) = best else { return false };
        let (ia, ib, ic) = (idx[(k + m - 1) % m], idx[k], idx[(k + 1) % m]);
        out.push([lp[ia], lp[ib], lp[ic]]);
        idx.remove(k);
        let m = idx.len();
        // the neighbours have new neighbours; the ears the clipped vertex spoiled may be free now
        let near = [(k + m - 1) % m, k % m];
        for p in 0..m {
            let v = idx[p];
            if near.contains(&p) || matches!(ears[v], Ear::Blocked(j) if j == ib) {
                ears[v] = judge_ear(&idx, p);
            }
        }
    }
    if let [a, b, c] = idx.as_slice() {
        out.push([lp[*a], lp[*b], lp[*c]]);
    }
    mesh.triangles.extend(out);
    true
}

/// Fan around a new centre vertex (fallback for loops that do not project to a simple polygon).
pub(crate) fn fan_fill(mesh: &mut Mesh, lp: &[u32]) {
    let (o, _, _, _) = loop_frame(mesh, lp);
    let c = mesh.positions.len() as u32;
    mesh.positions.push(o);
    for k in 0..lp.len() {
        mesh.triangles.push([lp[k], lp[(k + 1) % lp.len()], c]);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{Rng, tri};

    /// The original rule, ears judged afresh each round, as the reference.
    fn clip_naive(p2: &[[f64; 2]], sign: f64) -> Option<Vec<[usize; 3]>> {
        let mut idx: Vec<usize> = (0..p2.len()).collect();
        let mut out = Vec::new();
        while idx.len() > 3 {
            let m = idx.len();
            let mut best: Option<(usize, f64)> = None;
            for k in 0..m {
                if let Ear::Valid(q) = judge(p2, &idx, k, sign)
                    && best.is_none_or(|(_, bq)| q > bq)
                {
                    best = Some((k, q));
                }
            }
            let (k, _) = best?;
            out.push([idx[(k + m - 1) % m], idx[k], idx[(k + 1) % m]]);
            idx.remove(k);
        }
        out.push([idx[0], idx[1], idx[2]]);
        Some(out)
    }

    /// A wavy star-shaped loop in the plane (simple, with reflex vertices).
    fn star(n: usize, seed: u64) -> Mesh {
        let mut rng = Rng::new(seed);
        let positions = (0..n)
            .map(|k| {
                let a = std::f64::consts::TAU * k as f64 / n as f64;
                let r = 10.0 + 3.0 * (7.0 * a).sin() + rng.range(-0.4, 0.4);
                Point3::new(r * a.cos(), r * a.sin(), 0.0)
            })
            .collect();
        Mesh { positions, triangles: Vec::new() }
    }

    #[test]
    fn the_kept_judgements_give_the_triangles_of_judging_afresh() {
        for (n, seed) in [(12, 1), (60, 2), (240, 3)] {
            let mut m = star(n, seed);
            let lp: Vec<u32> = (0..n as u32).collect();
            let (o, x, y, _) = loop_frame(&m, &lp);
            let p2: Vec<[f64; 2]> = m
                .positions
                .iter()
                .map(|p| {
                    let d = p - o;
                    [d.dot(&x), d.dot(&y)]
                })
                .collect();
            let area2: f64 = (0..n).map(|k| p2[k][0] * p2[(k + 1) % n][1] - p2[(k + 1) % n][0] * p2[k][1]).sum();
            let want = clip_naive(&p2, if area2 >= 0.0 { 1.0 } else { -1.0 }).unwrap();
            assert!(ear_clip(&mut m, &lp));
            let got: Vec<[usize; 3]> = m.triangles.iter().map(|t| t.map(|v| v as usize)).collect();
            assert_eq!(got, want, "n = {n}");
            let area: f64 = (0..m.triangles.len())
                .map(|t| {
                    tri::area(
                        &m.positions[m.triangles[t][0] as usize],
                        &m.positions[m.triangles[t][1] as usize],
                        &m.positions[m.triangles[t][2] as usize],
                    )
                })
                .sum();
            assert!(area > 0.0 && m.triangles.len() == n - 2);
        }
    }

    #[test]
    fn a_long_loop_is_clipped_in_reasonable_time() {
        let mut m = star(20_000, 4);
        let lp: Vec<u32> = (0..20_000).collect();
        let t = std::time::Instant::now();
        assert!(ear_clip(&mut m, &lp));
        assert_eq!(m.triangles.len(), 19_998);
        assert!(t.elapsed().as_secs() < 60, "{:?}", t.elapsed());
    }
}
