//! Pose-graph relaxation of several scans at once: every pair pose is a measurement, and all poses
//! move together so that the disagreement around loops is spread over the loop instead of piling
//! up at its far end.
//!
//! An edge `(i, j)` carries the measured motion `T` (scan `j` -> scan `i`) and a set of virtual
//! correspondences: points `p` of scan `j` that lie on scan `i` at the measured pose, paired with
//! their images `T p`. The cost is the weighted sum of the squared distances between the two
//! images of every point under the unknown poses `X`: `|X_i T p - X_j p|^2`. This is the
//! point-form network registration of Lu and Milios (1997), with the virtual correspondences of
//! Pulli's multiview registration (1999); the minimisation is Gauss-Newton with Levenberg-Marquardt
//! damping on all poses at once (the normal equations are dense, 6 per scan), the pose of one scan
//! held fixed (Grisetti et al. 2010 describe the same scheme for general pose graphs). A Huber
//! weight per edge keeps one bad measurement from dragging the rest.
use crate::AlignError;
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use nalgebra::{DMatrix, DVector, Matrix3, Rotation3, SMatrix, Vector3};
use surfacecraft_geom::{Bvh, Cancel, Mesh, Point3, Rng, Transform};

/// Most virtual correspondences per edge.
pub(super) const MAX_VIRTUAL: usize = 300;
/// Fewest virtual correspondences for an edge to count as a measurement.
pub(super) const MIN_VIRTUAL: usize = 24;
/// Samples drawn from the moving scan to find the correspondences.
const DRAWN: usize = 1500;
/// Huber threshold, in tolerances: an edge whose RMS misfit is larger is down-weighted.
const HUBER: f64 = 2.0;
/// Outer iterations (each re-linearises and re-weights).
const MAX_ITERATIONS: usize = 40;
/// Damping attempts per iteration.
const MAX_DAMPING_TRIES: usize = 12;
/// The relaxation has converged when no point moves by more than this fraction of the extent.
const STEP_TOLERANCE: f64 = 1e-9;
/// Relative cost decrease below which the relaxation has converged.
const COST_TOLERANCE: f64 = 1e-10;

/// A pair pose as a measurement.
#[derive(Clone, Debug)]
pub struct Edge {
    /// The scan the measurement is expressed in (the fixed scan of the pair).
    pub i: usize,
    /// The scan that was moved onto `i`.
    pub j: usize,
    /// The measured motion, scan `j` -> scan `i`.
    pub measured: Transform,
    /// Strength of the measurement (overlap over squared residual).
    pub weight: f64,
    /// Distance (mm) within which the two scans count as coincident.
    pub tolerance: f64,
    /// Virtual correspondences: points of scan `j` that lie on scan `i` at the measured pose.
    pub points: Vec<Point3>,
}

/// How far an edge disagrees with a set of poses.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize, JsonSchema)]
pub struct Misfit {
    /// RMS distance (mm) between the two images of the edge's points (the two scans' common points, once by the edge's own pose and once by the poses of its two scans).
    pub rms_mm: f64,
    /// Angle (degrees) of the rotation that would make the two images agree.
    pub angle_degrees: f64,
}

/// The poses after relaxation.
#[derive(Clone, Debug)]
pub struct Relaxed {
    /// Scan -> common frame, identity for the fixed scan and for scans no edge touches.
    pub poses: Vec<Transform>,
    /// Iterations used.
    pub iterations: usize,
    /// Did it stop on the step or the cost tolerance (not on the iteration limit)?
    pub converged: bool,
}

/// The points of `moving` that lie within `tolerance` of `fixed` (through `bvh`) once moved by
/// `t`, at most [`MAX_VIRTUAL`], area-uniform and seeded.
pub(super) fn virtual_points(moving: &Mesh, bvh: &Bvh, t: &Transform, tolerance: f64, seed: u64) -> Vec<Point3> {
    moving
        .sample_points(DRAWN, &mut Rng::new(seed))
        .into_iter()
        .map(|(p, _)| p)
        .filter(|p| bvh.closest(&t.apply(p), tolerance).is_some())
        .take(MAX_VIRTUAL)
        .collect()
}

/// The disagreement of edge `e` with `poses`.
#[must_use]
pub fn misfit(e: &Edge, poses: &[Transform]) -> Misfit {
    let (Some(xi), Some(xj)) = (poses.get(e.i), poses.get(e.j)) else { return Misfit::default() };
    let via_i = xi.then_after(&e.measured);
    let sum: f64 = e.points.iter().map(|p| (via_i.apply(p) - xj.apply(p)).norm_squared()).sum();
    let rms_mm = (sum / e.points.len().max(1) as f64).sqrt();
    let q = via_i.inverse().then_after(xj).quaternion();
    Misfit { rms_mm, angle_degrees: (2.0 * q.vector().norm().atan2(q.scalar().abs())).to_degrees() }
}

fn skew(a: &Vector3<f64>) -> Matrix3<f64> {
    Matrix3::new(0.0, -a.z, a.y, a.z, 0.0, -a.x, -a.y, a.x, 0.0)
}

type Block = SMatrix<f64, 3, 6>;

/// Jacobians of `a - b` with respect to a small motion `(w, v)` of the pose behind `a` and the one
/// behind `b`: `a` moves by `w x a + v`, so the derivative is `[-[a]x | I]`; `b` enters negated.
fn jacobians(a: &Vector3<f64>, b: &Vector3<f64>) -> (Block, Block) {
    let (mut ji, mut jj) = (Block::zeros(), Block::zeros());
    ji.fixed_view_mut::<3, 3>(0, 0).copy_from(&(-skew(a)));
    ji.fixed_view_mut::<3, 3>(0, 3).copy_from(&Matrix3::identity());
    jj.fixed_view_mut::<3, 3>(0, 0).copy_from(&skew(b));
    jj.fixed_view_mut::<3, 3>(0, 3).copy_from(&(-Matrix3::identity()));
    (ji, jj)
}

/// The unknowns: a slot per scan that has an edge, except the fixed one.
fn slots(n: usize, root: usize, edges: &[Edge]) -> Vec<Option<usize>> {
    let mut used = vec![false; n];
    for e in edges {
        for k in [e.i, e.j] {
            if let Some(u) = used.get_mut(k) {
                *u = true;
            }
        }
    }
    let mut next = 0;
    (0..n)
        .map(|k| {
            if k == root || !used.get(k).copied().unwrap_or(false) {
                return None;
            }
            next += 1;
            Some(next - 1)
        })
        .collect()
}

/// Weight of every point of every edge for this iteration: the edge weight spread over its points,
/// times the Huber factor of the edge's current misfit.
fn point_weights(edges: &[Edge], poses: &[Transform]) -> Vec<f64> {
    let total: f64 = edges.iter().map(|e| e.weight).sum::<f64>().max(f64::MIN_POSITIVE);
    edges
        .iter()
        .map(|e| {
            let m = misfit(e, poses).rms_mm;
            let limit = HUBER * e.tolerance;
            let huber = if m > limit && m > 0.0 { limit / m } else { 1.0 };
            e.weight / total * huber / e.points.len().max(1) as f64
        })
        .collect()
}

struct System {
    h: DMatrix<f64>,
    g: DVector<f64>,
    cost: f64,
}

fn add_block(h: &mut DMatrix<f64>, r: Option<usize>, c: Option<usize>, m: &SMatrix<f64, 6, 6>) {
    if let (Some(r), Some(c)) = (r, c) {
        let mut view = h.fixed_view_mut::<6, 6>(6 * r, 6 * c);
        view += m;
    }
}

fn normal_equations(edges: &[Edge], poses: &[Transform], slot: &[Option<usize>], weights: &[f64], size: usize) -> System {
    let mut sys = System { h: DMatrix::zeros(size, size), g: DVector::zeros(size), cost: 0.0 };
    for (e, &w) in edges.iter().zip(weights) {
        let (Some(xi), Some(xj)) = (poses.get(e.i), poses.get(e.j)) else { continue };
        let (si, sj) = (slot.get(e.i).copied().flatten(), slot.get(e.j).copied().flatten());
        let via_i = xi.then_after(&e.measured);
        for p in &e.points {
            let (a, b) = (via_i.apply(p).coords, xj.apply(p).coords);
            let r = a - b;
            sys.cost += w * r.norm_squared();
            let (ji, jj) = jacobians(&a, &b);
            add_block(&mut sys.h, si, si, &(ji.transpose() * ji * w));
            add_block(&mut sys.h, sj, sj, &(jj.transpose() * jj * w));
            let cross = ji.transpose() * jj * w;
            add_block(&mut sys.h, si, sj, &cross);
            add_block(&mut sys.h, sj, si, &cross.transpose());
            for (s, j) in [(si, ji), (sj, jj)] {
                if let Some(s) = s {
                    let mut view = sys.g.fixed_rows_mut::<6>(6 * s);
                    view += j.transpose() * r * w;
                }
            }
        }
    }
    sys
}

/// Solves `(H + lambda diag(H)) d = -g`; `None` when the damped system is not positive definite.
fn solve_damped(sys: &System, lambda: f64) -> Option<DVector<f64>> {
    let mut h = sys.h.clone();
    for k in 0..h.nrows() {
        let d = sys.h[(k, k)];
        h[(k, k)] += lambda * d.max(1e-12) + 1e-12;
    }
    h.cholesky().map(|c| c.solve(&(-&sys.g)))
}

fn step_poses(poses: &[Transform], slot: &[Option<usize>], d: &DVector<f64>) -> Vec<Transform> {
    poses
        .iter()
        .zip(slot)
        .map(|(x, s)| match s {
            None => *x,
            Some(s) => match d.as_slice().get(6 * s..6 * s + 6) {
                Some(&[wx, wy, wz, vx, vy, vz]) => {
                    Transform { rotation: Rotation3::new(Vector3::new(wx, wy, wz)), translation: Vector3::new(vx, vy, vz) }
                        .then_after(x)
                }
                _ => *x,
            },
        })
        .collect()
}

fn extent(edges: &[Edge]) -> f64 {
    let bb = surfacecraft_geom::Aabb::from_points(edges.iter().flat_map(|e| e.points.iter()));
    if bb.is_empty() { 1.0 } else { bb.diagonal().max(f64::MIN_POSITIVE) }
}

/// Moves every pose by the damped Gauss-Newton step that lowers the cost, or `None` when no damping
/// does. Returns the new poses, the new cost and the largest displacement of a pose (mm at the
/// extent of the scene).
fn improve(
    edges: &[Edge],
    poses: &[Transform],
    slot: &[Option<usize>],
    size: usize,
    reach: f64,
) -> Option<(Vec<Transform>, f64, f64, f64)> {
    let weights = point_weights(edges, poses);
    let sys = normal_equations(edges, poses, slot, &weights, size);
    let mut lambda = 1e-9;
    for _ in 0..MAX_DAMPING_TRIES {
        if let Some(d) = solve_damped(&sys, lambda) {
            let next = step_poses(poses, slot, &d);
            let after = normal_equations(edges, &next, slot, &weights, size).cost;
            if after.is_finite() && after <= sys.cost {
                let step = d
                    .as_slice()
                    .chunks_exact(6)
                    .map(|c| {
                        let norm = |v: &[f64]| v.iter().map(|x| x * x).sum::<f64>().sqrt();
                        norm(c.get(..3).unwrap_or(&[])) * reach + norm(c.get(3..).unwrap_or(&[]))
                    })
                    .fold(0.0, f64::max);
                return Some((next, after, sys.cost, step));
            }
        }
        lambda *= 10.0;
    }
    None
}

/// Relaxes the poses of the `n` scans from `init` (scan -> common frame) against `edges`, holding
/// scan `root` where it is. Scans without an edge keep their pose.
///
/// # Errors
/// [`AlignError::Cancelled`] when `cancel` is raised between iterations.
pub fn relax(n: usize, root: usize, edges: &[Edge], init: &[Transform], cancel: &Cancel) -> Result<Relaxed, AlignError> {
    let slot = slots(n, root, edges);
    let size = 6 * slot.iter().flatten().count();
    let mut poses = init.to_vec();
    if size == 0 || edges.is_empty() {
        return Ok(Relaxed { poses, iterations: 0, converged: true });
    }
    let reach = extent(edges);
    let mut converged = false;
    let mut iterations = 0;
    while iterations < MAX_ITERATIONS && !converged {
        if cancel.is_cancelled() {
            return Err(AlignError::Cancelled);
        }
        iterations += 1;
        let Some((next, after, before, step)) = improve(edges, &poses, &slot, size, reach) else {
            converged = true;
            continue;
        };
        poses = next;
        converged = step <= STEP_TOLERANCE * reach || (before - after) <= COST_TOLERANCE * before.max(f64::MIN_POSITIVE);
    }
    Ok(Relaxed { poses, iterations, converged })
}
