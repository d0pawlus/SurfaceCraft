//! Mesh reduction with a measured, verified tolerance.
//!
//! [`reduce_to_tolerance`] runs the quadric edge-collapse decimation of [`crate::edit`] with a
//! per-vertex error budget (every collapse is measured against the original surface through a
//! [`Bvh`], so noisy data reduces as far as it really can), then measures the result against the
//! original in both directions:
//! every original vertex to the reduced surface, and every reduced vertex and triangle centroid to
//! the original surface. Where the tolerance is exceeded the budget of the original vertices around
//! the offending samples is halved (detail is kept there) and the reduction is repeated from the
//! original, until the wanted share of the samples is within the tolerance in both directions.
//! Everything is deterministic: the decimation, the measurement (results in index order) and the
//! marking of the vertices to refine do not depend on threads.
use crate::edit::{DecimateOptions, Guard, decimate_guarded, vertex_rings};
use crate::{Bvh, Mesh, Point3, par};

/// The starting budget as a fraction of the tolerance: collapses are measured one at a time, so the
/// error of many of them can add up; the refinement rounds fix what remains.
const START_FRACTION: f64 = 0.8;
/// Budget factor applied around a violating sample in each refinement round.
const TIGHTEN: f64 = 0.5;
/// Rings of original vertices refined around a violating sample.
const RING_DEPTH: usize = 2;
/// No sample may be farther than this many tolerances, whatever the share (the stop condition is the
/// share within the tolerance and this hard cap).
pub const HARD_CAP: f64 = 2.0;

/// What to guarantee.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ToleranceOptions {
    /// Distance (mm) the surface may move.
    pub tolerance: f64,
    /// Share of the samples that must be within `tolerance` in both directions, in `(0, 1]`.
    pub share: f64,
    /// Refinement rounds at most (the reduction is repeated once per round).
    pub max_iterations: usize,
}

impl ToleranceOptions {
    /// `tolerance` mm for 99.9 % of the samples, at most 16 rounds.
    #[must_use]
    pub fn new(tolerance: f64) -> Self {
        Self { tolerance, share: 0.999, max_iterations: 16 }
    }
}

/// Distances of one direction of the measurement.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct DeviationStats {
    /// Number of samples.
    pub samples: usize,
    /// Largest distance (mm).
    pub max: f64,
    /// 99th percentile distance (mm).
    pub p99: f64,
    /// Share of the samples within the tolerance, in `[0, 1]`.
    pub share_within: f64,
}

/// What the reduction measured.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ToleranceReport {
    /// Triangles of the input.
    pub triangles_before: usize,
    /// Triangles of the result.
    pub triangles_after: usize,
    /// Original vertices to the reduced surface.
    pub original_to_reduced: DeviationStats,
    /// Reduced vertices and triangle centroids to the original surface.
    pub reduced_to_original: DeviationStats,
    /// Reduction rounds run (1 when the first reduction already met the tolerance).
    pub iterations: usize,
    /// Did the result meet the share and the hard cap? `false` when the rounds ran out.
    pub met: bool,
}

struct Measured {
    forward: Vec<f64>,
    backward: Vec<f64>,
    /// Original triangle nearest to each backward sample.
    backward_tri: Vec<u32>,
}

fn measure(orig: &Mesh, used: &[bool], bvh_orig: &Bvh, reduced: &Mesh) -> Measured {
    let bvh_red = Bvh::build(reduced);
    let sampled: Vec<u32> = (0..orig.positions.len() as u32).filter(|&i| used.get(i as usize) == Some(&true)).collect();
    let forward = par::map(sampled.len(), |k| {
        let p = sampled.get(k).and_then(|&i| orig.positions.get(i as usize)).copied().unwrap_or_else(Point3::origin);
        bvh_red.closest(&p, f64::INFINITY).map_or(f64::MAX, |h| h.distance)
    });
    let nv = reduced.positions.len();
    let hits = par::map(nv + reduced.triangles.len(), |i| {
        let p = if i < nv { reduced.positions.get(i).copied() } else { reduced.face_centroid(i - nv) }
            .unwrap_or_else(Point3::origin);
        bvh_orig.closest(&p, f64::INFINITY).map_or((f64::MAX, u32::MAX), |h| (h.distance, h.triangle as u32))
    });
    let (backward, backward_tri) = hits.into_iter().unzip();
    // forward samples keep the original vertex index through `sampled`; the caller re-derives it
    Measured { forward, backward, backward_tri }
}

fn stats(d: &[f64], tolerance: f64) -> DeviationStats {
    let n = d.len();
    if n == 0 {
        return DeviationStats { samples: 0, max: 0.0, p99: 0.0, share_within: 1.0 };
    }
    let max = d.iter().copied().fold(0.0, f64::max);
    let within = d.iter().filter(|&&x| x <= tolerance).count();
    let mut v = d.to_vec();
    let k = (((n as f64) * 0.99).ceil() as usize).clamp(1, n) - 1;
    let (_, p99, _) = v.select_nth_unstable_by(k, f64::total_cmp);
    DeviationStats { samples: n, max, p99: *p99, share_within: within as f64 / n as f64 }
}

/// Halves the budget of `seed` and of the vertices within [`RING_DEPTH`] rings of it, once per round.
fn tighten(seed: u32, ring: &[Vec<u32>], budget: &mut [f64], mark: &mut [u32], round: u32) {
    let mut frontier = vec![seed];
    for _ in 0..=RING_DEPTH {
        let mut next = Vec::new();
        for &v in &frontier {
            let Some(m) = mark.get_mut(v as usize) else { continue };
            if *m == round {
                continue;
            }
            *m = round;
            if let Some(b) = budget.get_mut(v as usize) {
                *b *= TIGHTEN;
            }
            if let Some(r) = ring.get(v as usize) {
                next.extend_from_slice(r);
            }
        }
        frontier = next;
    }
}

/// Reduces `mesh` with [`DecimateOptions`] (its `target_triangles` is a floor, 0 = as far as the
/// tolerance allows) so that at least `tol.share` of the samples are within `tol.tolerance` in both
/// directions and none is farther than [`HARD_CAP`] tolerances. The mesh is replaced by the last
/// reduction; the report says whether the requirement was met. `None` (mesh untouched) for an invalid
/// mesh or a tolerance or share outside their range.
pub fn reduce_to_tolerance(mesh: &mut Mesh, opts: &DecimateOptions, tol: &ToleranceOptions) -> Option<ToleranceReport> {
    reduce_to_tolerance_scaled(mesh, opts, tol, &[])
}

/// [`reduce_to_tolerance`] with a starting budget per vertex of the input: `scale[v]` multiplies the
/// starting budget of vertex `v` (1.0 where `scale` is shorter than the mesh), so detail can be
/// asked for where a later stage found the reduction too coarse. A scale that is not a positive
/// finite number counts as 1.0. The refinement rounds are the same as without a scale.
pub fn reduce_to_tolerance_scaled(
    mesh: &mut Mesh,
    opts: &DecimateOptions,
    tol: &ToleranceOptions,
    scale: &[f64],
) -> Option<ToleranceReport> {
    let ok_range = tol.tolerance.is_finite() && tol.tolerance > 0.0 && tol.share > 0.0 && tol.share <= 1.0;
    if !ok_range || mesh.validate().is_err() {
        return None;
    }
    let orig = mesh.clone();
    let nv = orig.positions.len();
    let mut used = vec![false; nv];
    for t in &orig.triangles {
        for &v in t {
            if let Some(u) = used.get_mut(v as usize) {
                *u = true;
            }
        }
    }
    let sampled: Vec<u32> = (0..nv as u32).filter(|&i| used.get(i as usize) == Some(&true)).collect();
    let bvh_orig = Bvh::build(&orig);
    let start = tol.tolerance * START_FRACTION;
    let mut budget: Vec<f64> =
        (0..nv).map(|v| start * scale.get(v).copied().filter(|x| x.is_finite() && *x > 0.0).unwrap_or(1.0)).collect();
    let mut mark = vec![0u32; nv];
    let mut ring: Option<Vec<Vec<u32>>> = None;
    let mut report = ToleranceReport { triangles_before: orig.triangles.len(), ..ToleranceReport::default() };
    let rounds = tol.max_iterations.max(1);
    for round in 1..=rounds {
        let mut cand = orig.clone();
        let mut working = budget.clone();
        decimate_guarded(&mut cand, opts, Some(Guard { bvh: &bvh_orig, budget: &mut working }));
        let m = measure(&orig, &used, &bvh_orig, &cand);
        let forward = stats(&m.forward, tol.tolerance);
        let backward = stats(&m.backward, tol.tolerance);
        let cap = HARD_CAP * tol.tolerance;
        let met = forward.share_within >= tol.share
            && backward.share_within >= tol.share
            && forward.max <= cap
            && backward.max <= cap;
        report = ToleranceReport {
            triangles_before: orig.triangles.len(),
            triangles_after: cand.triangles.len(),
            original_to_reduced: forward,
            reduced_to_original: backward,
            iterations: round,
            met,
        };
        *mesh = cand;
        if met || round == rounds {
            break;
        }
        // keep detail around every sample that is too far: the original vertex itself, or the
        // original triangle nearest to a reduced vertex or centroid
        let rings = ring.get_or_insert_with(|| vertex_rings(&orig));
        let r = round as u32;
        for (k, &d) in m.forward.iter().enumerate() {
            if d > tol.tolerance
                && let Some(&v) = sampled.get(k)
            {
                tighten(v, rings, &mut budget, &mut mark, r);
            }
        }
        for (k, &d) in m.backward.iter().enumerate() {
            if d > tol.tolerance
                && let Some(t) = m.backward_tri.get(k).and_then(|&t| orig.triangles.get(t as usize))
            {
                for &v in t {
                    tighten(v, rings, &mut budget, &mut mark, r);
                }
            }
        }
    }
    Some(report)
}

/// Detail levels after one more stage: `levels[v] + 1` (at most `max_level`) for every seed vertex
/// and its neighbours up to `depth` rings away, the others unchanged. A vertex index outside the
/// mesh is ignored; `levels` shorter than the mesh counts as zeros. A mesh whose triangles name
/// vertices it does not have gets its levels back unchanged.
#[must_use]
pub fn raise_levels(mesh: &Mesh, seeds: &[u32], levels: &[u8], depth: usize, max_level: u8) -> Vec<u8> {
    let nv = mesh.positions.len();
    let mut out: Vec<u8> = (0..nv).map(|v| levels.get(v).copied().unwrap_or(0)).collect();
    if mesh.triangles.iter().flatten().any(|&v| v as usize >= nv) {
        return out;
    }
    let rings = vertex_rings(mesh);
    let mut reached = vec![false; nv];
    let mut frontier: Vec<u32> = Vec::new();
    for &v in seeds {
        if let Some(r) = reached.get_mut(v as usize)
            && !*r
        {
            *r = true;
            frontier.push(v);
        }
    }
    let mut all = frontier.clone();
    for _ in 0..depth {
        let mut next = Vec::new();
        for &v in &frontier {
            for &w in rings.get(v as usize).map_or(&[][..], Vec::as_slice) {
                if let Some(r) = reached.get_mut(w as usize)
                    && !*r
                {
                    *r = true;
                    next.push(w);
                }
            }
        }
        all.extend_from_slice(&next);
        frontier = next;
    }
    for v in all {
        if let Some(l) = out.get_mut(v as usize) {
            *l = l.saturating_add(1).min(max_level);
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{Rng, Vec3, synth};

    fn one_sided(a: &Mesh, b: &Mesh) -> Vec<f64> {
        let bvh = Bvh::build(b);
        a.positions.iter().map(|p| bvh.closest(p, f64::INFINITY).map_or(f64::MAX, |h| h.distance)).collect()
    }

    /// Independent check of the report: original vertices to the reduced mesh.
    fn assert_within(orig: &Mesh, red: &Mesh, tol: f64) {
        let d = one_sided(orig, red);
        let share = d.iter().filter(|&&x| x <= tol).count() as f64 / d.len() as f64;
        let max = d.iter().copied().fold(0.0, f64::max);
        assert!(share >= 0.999, "share {share}");
        assert!(max <= 2.0 * tol, "max {max}");
    }

    #[test]
    fn sphere_meets_tolerance_both_ways() {
        let orig = synth::sphere(Point3::origin(), 20.0, 410, 250);
        assert!(orig.triangle_count() >= 200_000);
        let tol = 0.05;
        let mut m = orig.clone();
        let rep = reduce_to_tolerance(&mut m, &DecimateOptions::default(), &ToleranceOptions::new(tol)).unwrap();
        eprintln!("sphere: {rep:?}");
        assert!(rep.met, "{rep:?}");
        assert!(rep.original_to_reduced.share_within >= 0.999);
        assert!(rep.reduced_to_original.share_within >= 0.999);
        assert!(rep.original_to_reduced.max <= 2.0 * tol && rep.reduced_to_original.max <= 2.0 * tol, "{rep:?}");
        assert!(rep.triangles_after * 5 <= rep.triangles_before, "{rep:?}");
        assert_eq!(rep.triangles_after, m.triangle_count());
        assert_within(&orig, &m, tol);
        let back = one_sided(&m, &orig);
        assert!(back.iter().copied().fold(0.0, f64::max) <= 2.0 * tol);
    }

    #[test]
    fn deterministic() {
        let orig = synth::sphere(Point3::origin(), 10.0, 96, 48);
        let (mut a, mut b) = (orig.clone(), orig);
        let o = DecimateOptions::default();
        let t = ToleranceOptions::new(0.05);
        let ra = reduce_to_tolerance(&mut a, &o, &t).unwrap();
        let rb = reduce_to_tolerance(&mut b, &o, &t).unwrap();
        assert_eq!(ra, rb);
        assert_eq!(a, b);
    }

    #[test]
    fn fillets_stay_within_tolerance() {
        let (half, r, tol) = (15.0, 2.0, 0.05);
        let orig = synth::rounded_cuboid(
            Point3::new(-half, -half, -half),
            Vec3::new(2.0 * half, 2.0 * half, 2.0 * half),
            r,
            80,
        );
        let mut m = orig.clone();
        let rep = reduce_to_tolerance(&mut m, &DecimateOptions::default(), &ToleranceOptions::new(tol)).unwrap();
        eprintln!("box: {rep:?}");
        assert!(rep.met, "{rep:?}");
        assert!(rep.triangles_after * 5 <= rep.triangles_before, "{rep:?}");
        assert_within(&orig, &m, tol);
        // every reduced vertex lies on the analytic rounded box (the fillets were not cut)
        let inner = half - r;
        let mut on_fillet = 0;
        for p in &m.positions {
            let c = Point3::new(p.x.clamp(-inner, inner), p.y.clamp(-inner, inner), p.z.clamp(-inner, inner));
            let dist = ((p - c).norm() - r).abs();
            assert!(dist <= 1.5 * tol, "vertex {p} is {dist} off the analytic surface");
            if (p - c).norm() > 0.0 && (p.x.abs() > inner || p.y.abs() > inner || p.z.abs() > inner) {
                let beyond = [p.x, p.y, p.z].iter().filter(|v| v.abs() > inner + 1e-9).count();
                if beyond >= 2 {
                    on_fillet += 1;
                }
            }
        }
        // the 12 edge fillets keep several vertices each
        assert!(on_fillet >= 12 * 3, "only {on_fillet} vertices on the fillets");
    }

    #[test]
    fn noisy_plane_reduces_a_lot() {
        let mut orig = synth::plane(Point3::origin(), Vec3::x(), Vec3::y(), 100.0, 100.0, 200, 200);
        synth::add_noise(&mut orig, 0.01, &mut Rng::new(5));
        let mut m = orig.clone();
        let rep = reduce_to_tolerance(&mut m, &DecimateOptions::default(), &ToleranceOptions::new(0.05)).unwrap();
        eprintln!("plane: {rep:?}");
        assert!(rep.met, "{rep:?}");
        assert!(rep.triangles_after * 50 <= rep.triangles_before, "{rep:?}");
        assert_within(&orig, &m, 0.05);
    }

    #[test]
    fn creases_of_a_bracket_are_kept() {
        let orig = synth::refine(&synth::bracket(), |_| 1.5);
        let mut m = orig.clone();
        m.weld(1e-9);
        let before = m.triangle_count();
        let tol = 0.05;
        let rep = reduce_to_tolerance(&mut m, &DecimateOptions::default(), &ToleranceOptions::new(tol)).unwrap();
        eprintln!("bracket: {rep:?}");
        assert!(rep.met, "{rep:?}");
        assert_eq!(rep.triangles_before, before);
        assert!(rep.triangles_after * 3 <= before, "{rep:?}");
        assert_within(&orig, &m, tol);
    }

    #[test]
    fn scaled_start_budget_keeps_more_detail_where_asked() {
        let orig = synth::sphere(Point3::origin(), 20.0, 96, 48);
        let o = DecimateOptions::default();
        let t = ToleranceOptions::new(0.2);
        let mut plain = orig.clone();
        reduce_to_tolerance(&mut plain, &o, &t).unwrap();
        let fine: Vec<f64> = orig.positions.iter().map(|p| if p.x > 0.0 { 0.1 } else { 1.0 }).collect();
        let mut scaled = orig.clone();
        reduce_to_tolerance_scaled(&mut scaled, &o, &t, &fine).unwrap();
        let near = |m: &Mesh| m.positions.iter().filter(|p| p.x > 0.0).count();
        assert!(near(&scaled) > near(&plain), "{} vs {}", near(&scaled), near(&plain));
        let mut again = orig.clone();
        reduce_to_tolerance_scaled(&mut again, &o, &t, &fine).unwrap();
        assert_eq!(again, scaled);
        let mut hostile = orig.clone();
        reduce_to_tolerance_scaled(&mut hostile, &o, &t, &[f64::NAN, -1.0, 0.0, f64::INFINITY]).unwrap();
        assert_eq!(hostile, plain);
    }

    #[test]
    fn raise_levels_reaches_the_rings_and_saturates() {
        let m = synth::sphere(Point3::origin(), 5.0, 16, 8);
        let up = raise_levels(&m, &[0, 9999], &[], 1, 2);
        assert_eq!(up.len(), m.positions.len());
        assert_eq!(up[0], 1);
        let touched = up.iter().filter(|&&l| l > 0).count();
        assert!(touched > 1 && touched < m.positions.len());
        let twice = raise_levels(&m, &[0], &raise_levels(&m, &[0], &up, 1, 2), 1, 2);
        assert_eq!(twice[0], 2);
        let broken = Mesh { positions: vec![Point3::origin()], triangles: vec![[0, 1, 2]] };
        assert_eq!(raise_levels(&broken, &[0], &[3], 2, 4), vec![3]);
    }

    #[test]
    fn bad_input_is_refused() {
        let mut m = synth::sphere(Point3::origin(), 5.0, 16, 8);
        let o = DecimateOptions::default();
        for t in [0.0, -1.0, f64::NAN, f64::INFINITY] {
            assert!(reduce_to_tolerance(&mut m, &o, &ToleranceOptions::new(t)).is_none());
        }
        let bad = ToleranceOptions { share: 1.5, ..ToleranceOptions::new(0.1) };
        assert!(reduce_to_tolerance(&mut m, &o, &bad).is_none());
        let mut broken = Mesh { positions: vec![Point3::origin()], triangles: vec![[0, 1, 7]] };
        assert!(reduce_to_tolerance(&mut broken, &o, &ToleranceOptions::new(0.1)).is_none());
        // an unreachable tolerance keeps (nearly) everything and still reports honestly
        let mut noisy = synth::sphere(Point3::origin(), 5.0, 32, 16);
        synth::add_noise(&mut noisy, 0.05, &mut Rng::new(2));
        let before = noisy.triangle_count();
        let rep = reduce_to_tolerance(&mut noisy, &o, &ToleranceOptions::new(1e-6)).unwrap();
        assert!(rep.met && rep.triangles_after * 10 >= before * 9, "{rep:?}");
    }
}
