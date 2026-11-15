//! ICP, point-to-plane (Chen & Medioni 1991), on a BVH of the target surface.
//!
//! Each iteration pairs every source sample with its closest point on the target surface,
//! drops pairs farther than a trimming distance, and solves the linearised point-to-plane
//! least squares for a small motion (6 × 6 normal equations, Levenberg-damped when the target
//! slides, e.g. a plane). The motion is applied exactly (axis-angle). It stops when the largest
//! displacement of any sample between two iterations falls below `tolerance` (design doc §11.1:
//! 1e-4 mm per iteration).
use crate::AlignError;
use nalgebra::{Matrix6, Vector6};
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use surfacecraft_geom::{Bvh, Cancel, Mesh, Point3, SurfaceHit, Transform, Vec3};

/// ICP settings.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(default)]
pub struct IcpOptions {
    /// Maximum iterations.
    pub max_iterations: usize,
    /// Convergence: stop when no sample moved more than this between iterations (mm).
    pub tolerance: f64,
    /// Ignore pairs farther apart than this (mm); `None` = adaptive (3 × median, at least 10 × tolerance).
    pub max_distance: Option<f64>,
    /// Maximum number of source vertices used (evenly strided).
    pub max_samples: usize,
}

impl Default for IcpOptions {
    fn default() -> Self {
        Self { max_iterations: 100, tolerance: 1e-4, max_distance: None, max_samples: 20_000 }
    }
}

/// What an ICP run did.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
pub struct IcpReport {
    /// Final motion (source → target), including the initial guess.
    #[schemars(with = "[[f64; 4]; 4]")]
    pub transform: Transform,
    /// Iterations run.
    pub iterations: usize,
    /// Did the per-iteration change drop below the tolerance?
    pub converged: bool,
    /// RMS point-to-surface distance of the kept pairs at the end (mm).
    pub rms: f64,
    /// Largest sample displacement of the last iteration (mm).
    pub last_delta: f64,
    /// Per-iteration largest displacement (mm).
    pub deltas: Vec<f64>,
    /// Pairs kept in the last iteration.
    pub pairs: usize,
    /// Source samples the fit used (the vertices, evenly strided down to `max_samples`).
    pub samples: usize,
}

fn samples(source: &Mesh, max: usize) -> Vec<Point3> {
    let n = source.vertex_count();
    if n == 0 {
        return Vec::new();
    }
    let stride = n.div_ceil(max.max(1)).max(1);
    source.positions.iter().step_by(stride).copied().collect()
}

/// Exact rotation for the small rotation vector `w`.
fn rot(w: &Vec3) -> Transform {
    let a = w.norm();
    if a < 1e-300 { Transform::identity() } else { Transform::rotation_axis_angle(&(w / a), a) }
}

/// Aligns `source` onto the surface indexed by `target`, starting from `init`.
pub fn icp(source: &Mesh, target: &Bvh, init: &Transform, opts: &IcpOptions) -> Result<IcpReport, AlignError> {
    icp_cancellable(source, target, init, opts, &Cancel::new())
}

/// [`icp`] that gives up with [`AlignError::Cancelled`] when `cancel` is set (checked every
/// iteration).
pub fn icp_cancellable(
    source: &Mesh,
    target: &Bvh,
    init: &Transform,
    opts: &IcpOptions,
    cancel: &Cancel,
) -> Result<IcpReport, AlignError> {
    icp_impl(source, target, init, opts, cancel, &|h| target.triangle_normal(h.triangle), false)
}

/// [`icp_cancellable`] with the surface normal at each correspondence given by `normal` instead of
/// the normal of the triangle hit. On a fine noisy scan the normal of a single triangle tilts by
/// about the noise over the edge length, and where the target slides (a flat patch on a plane) the
/// tilts alone decide the motion; a smoothed normal does not. Also, a step that would move a sample
/// by more than the trimming distance is damped (a patch whose normals are all alike has an
/// almost singular system, and the solution would throw it off the target).
pub fn icp_with_normals(
    source: &Mesh,
    target: &Bvh,
    init: &Transform,
    opts: &IcpOptions,
    cancel: &Cancel,
    normal: &(dyn Fn(&SurfaceHit) -> Option<Vec3> + Sync),
) -> Result<IcpReport, AlignError> {
    icp_impl(source, target, init, opts, cancel, normal, true)
}

/// One surface hit of a sample: the closest point, the surface normal there and the distance.
type Hit = (Point3, Vec3, f64);

/// What stays fixed during an ICP run: the samples, the target and the settings.
struct IcpContext<'a> {
    pts: &'a [Point3],
    target: &'a Bvh,
    normal: &'a (dyn Fn(&SurfaceHit) -> Option<Vec3> + Sync),
    max_distance: Option<f64>,
    tol: f64,
    limit_step: bool,
}

/// What changes from iteration to iteration of an ICP run and ends up in the report.
struct IcpState {
    t: Transform,
    deltas: Vec<f64>,
    converged: bool,
    rms: f64,
    pairs: usize,
    lambda: f64,
}

/// The correspondences of one iteration: the moved samples, their hits and the trimming distance.
struct Pairing {
    moved: Vec<Point3>,
    hits: Vec<Option<Hit>>,
    cut: f64,
}

/// The normal equations of one iteration with the rotation centre and the number of kept pairs.
struct NormalEq {
    ata: Matrix6<f64>,
    atb: Vector6<f64>,
    sq: f64,
    pairs: usize,
    c: Vec3,
}

/// The result of the damped solve: the solution (if any), whether it is within reach, the damping.
struct Damped {
    x: Option<Vector6<f64>>,
    within: bool,
    lam: f64,
}

impl IcpContext<'_> {
    /// Pairs every sample moved by `t` with its closest surface point and fixes the trimming distance.
    fn correspondences(&self, t: &Transform) -> Result<Pairing, AlignError> {
        let (target, normal) = (self.target, self.normal);
        let moved: Vec<Point3> = self.pts.iter().map(|p| t.apply(p)).collect();
        let hits: Vec<Option<Hit>> = surfacecraft_geom::par::map_slice(&moved, |p| {
            target.closest(p, f64::INFINITY).and_then(|h| normal(&h).map(|n| (h.point, n, h.distance)))
        });
        let mut dists: Vec<f64> = hits.iter().flatten().map(|h| h.2).collect();
        if dists.len() < 6 {
            return Err(AlignError::NotEnough("too few correspondences".into()));
        }
        dists.sort_by(f64::total_cmp);
        let median = dists.get(dists.len() / 2).copied().unwrap_or(0.0);
        let tol = self.tol;
        let cut = self.max_distance.filter(|d| d.is_finite() && *d > 0.0).unwrap_or((3.0 * median).max(10.0 * tol));
        Ok(Pairing { moved, hits, cut })
    }

    /// One iteration: pair, solve, move. `Ok(true)` goes on, `Ok(false)` ends the run.
    fn iterate(&self, st: &mut IcpState) -> Result<bool, AlignError> {
        let pairing = self.correspondences(&st.t)?;
        let eq = normal_equations(&pairing)?;
        st.pairs = eq.pairs;
        st.rms = (eq.sq / eq.pairs as f64).sqrt();
        let sol = solve_damped(&eq, &pairing, self.limit_step, st.lambda);
        if !self.limit_step {
            st.lambda = sol.lam;
        }
        let Some(x) = sol.x else {
            return Err(AlignError::Degenerate("the point-to-plane system is singular".into()));
        };
        if !sol.within {
            // even the heaviest damping leaves a step beyond the trimming distance: the pose is not
            // moved on a step that would throw it off the target, and it has not converged
            return Ok(false);
        }
        let undamped = sol.lam == 0.0;
        let (next, delta) = apply_step(&st.t, &x, eq.c, self.pts);
        st.t = next;
        st.deltas.push(delta);
        if delta < self.tol && (undamped || !self.limit_step) {
            st.converged = true;
            return Ok(false);
        }
        Ok(true)
    }
}

/// The normal equations of r = n·(p − q) + (p × n)·w + n·t over the pairs within the trimming distance.
fn normal_equations(pairing: &Pairing) -> Result<NormalEq, AlignError> {
    let (moved, cut) = (&pairing.moved, pairing.cut);
    let mut ata = Matrix6::<f64>::zeros();
    let mut atb = Vector6::<f64>::zeros();
    let mut sq = 0.0;
    let mut pairs = 0;
    // centre the rotation at the sample centroid for conditioning
    let c: Vec3 = moved.iter().map(|p| p.coords).sum::<Vec3>() / moved.len() as f64;
    for (p, h) in moved.iter().zip(&pairing.hits) {
        let Some((q, n, d)) = h else { continue };
        if *d > cut {
            continue;
        }
        let pc = p.coords - c;
        let cr = pc.cross(n);
        let row = Vector6::new(cr.x, cr.y, cr.z, n.x, n.y, n.z);
        let r = n.dot(&(p - q));
        ata += row * row.transpose();
        atb += row * r;
        sq += d * d;
        pairs += 1;
    }
    if pairs < 6 {
        return Err(AlignError::NotEnough("too few pairs within the trimming distance".into()));
    }
    Ok(NormalEq { ata, atb, sq, pairs, c })
}

/// The largest displacement a solution `x` gives any of the moved samples (rotation about `c`).
fn reach(x: &Vector6<f64>, moved: &[Point3], c: Vec3) -> f64 {
    let (w, tr) = (Vec3::new(x[0], x[1], x[2]), Vec3::new(x[3], x[4], x[5]));
    moved.iter().map(|p| (w.cross(&(p.coords - c)) + tr).norm()).fold(0.0, f64::max)
}

/// Solves the normal equations, Levenberg-damped where the system is singular (sliding directions),
/// and where it is so nearly singular that the solution would move a sample by more than the
/// trimming distance (a patch that nothing holds, whose normals differ by noise only).
fn solve_damped(eq: &NormalEq, pairing: &Pairing, limit_step: bool, lambda: f64) -> Damped {
    let scale = eq.ata.diagonal().max().max(1e-300);
    // the damping of the plain entry point persists from iteration to iteration (as it always
    // did); with a limited step it starts from zero every iteration, so one damped step does
    // not slow all the later ones
    let mut lam = if limit_step { 0.0 } else { lambda };
    let mut last = None;
    let mut within = false;
    let sol = loop {
        let damped = eq.ata + Matrix6::identity() * (lam * scale);
        if let Some(ch) = damped.cholesky() {
            let x = ch.solve(&(-eq.atb));
            within = !limit_step || reach(&x, &pairing.moved, eq.c) <= pairing.cut;
            last = Some(x);
            if within {
                break last;
            }
        }
        lam = if lam == 0.0 { 1e-12 } else { lam * 10.0 };
        if lam > 1.0 {
            break last;
        }
    };
    Damped { x: sol, within, lam }
}

/// Applies the motion `x` about the centroid `c` (p' = R(p − c) + c + tr) to `t`: the new pose and
/// the largest displacement of the samples.
fn apply_step(t: &Transform, x: &Vector6<f64>, c: Vec3, pts: &[Point3]) -> (Transform, f64) {
    let w = Vec3::new(x[0], x[1], x[2]);
    let tr = Vec3::new(x[3], x[4], x[5]);
    let step = Transform::translation(c + tr).then_after(&rot(&w)).then_after(&Transform::translation(-c));
    let next = step.then_after(t);
    let delta = next.max_displacement(t, pts);
    (next, delta)
}

fn icp_impl(
    source: &Mesh,
    target: &Bvh,
    init: &Transform,
    opts: &IcpOptions,
    cancel: &Cancel,
    normal: &(dyn Fn(&SurfaceHit) -> Option<Vec3> + Sync),
    limit_step: bool,
) -> Result<IcpReport, AlignError> {
    let pts = samples(source, opts.max_samples);
    if pts.len() < 6 {
        return Err(AlignError::NotEnough(format!("{} source points; at least 6 are needed", pts.len())));
    }
    if target.is_empty() {
        return Err(AlignError::NotEnough("the target has no triangles".into()));
    }
    let tol = if opts.tolerance.is_finite() && opts.tolerance > 0.0 { opts.tolerance } else { 1e-4 };
    let ctx = IcpContext { pts: &pts, target, normal, max_distance: opts.max_distance, tol, limit_step };
    let mut st = IcpState { t: *init, deltas: Vec::new(), converged: false, rms: f64::INFINITY, pairs: 0, lambda: 0.0 };
    for _ in 0..opts.max_iterations.max(1) {
        if cancel.is_cancelled() {
            return Err(AlignError::Cancelled);
        }
        if !ctx.iterate(&mut st)? {
            break;
        }
    }
    Ok(IcpReport {
        transform: st.t,
        iterations: st.deltas.len(),
        converged: st.converged,
        rms: st.rms,
        last_delta: st.deltas.last().copied().unwrap_or(0.0),
        deltas: st.deltas,
        pairs: st.pairs,
        samples: pts.len(),
    })
}

/// RMS distance (mm) from `pts`, moved by `t`, to the surface indexed by `target`: how well a
/// motion fits a part of a scan. `None` when the target is empty or there are no points.
#[must_use]
pub fn rms_to_surface(target: &Bvh, pts: &[Point3], t: &Transform) -> Option<f64> {
    if target.is_empty() {
        return None;
    }
    let sq: Vec<Option<f64>> = surfacecraft_geom::par::map_slice(pts, |p| {
        target.closest(&t.apply(p), f64::INFINITY).map(|h| h.distance * h.distance)
    });
    let (sum, n) = sq.iter().flatten().fold((0.0_f64, 0_usize), |(s, n), d| (s + *d, n + 1));
    (n > 0).then(|| (sum / n as f64).sqrt())
}

#[cfg(test)]
mod tests {
    use super::*;
    use surfacecraft_geom::{Rng, synth};

    fn perturbation() -> Transform {
        Transform::rotation_axis_angle(&Vec3::new(0.2, 0.9, -0.4), 4.0_f64.to_radians())
            .then_after(&Transform::translation(Vec3::new(1.5, -2.0, 1.0)))
    }

    /// Gate M1 (design doc §11.1): a known motion is recovered on noise-free synthetic data to
    /// within 1e-6 mm, and ICP converges with Δ < 1e-4 mm per iteration.
    #[test]
    fn gate_m1_icp_recovers_known_motion_noise_free() {
        let target = synth::bracket();
        let truth = perturbation();
        // the scan is the part displaced by the inverse motion: ICP must find `truth`
        let source = target.transformed(&truth.inverse());
        let bvh = Bvh::build(&target);
        let rep = icp(&source, &bvh, &Transform::identity(), &IcpOptions::default()).unwrap();
        assert!(rep.converged, "{rep:?}");
        assert!(rep.last_delta < 1e-4);
        let err = rep.transform.max_displacement(&truth, &source.positions);
        eprintln!("gate_m1 icp: max error {err:e} mm, {} iterations, last delta {:e}", rep.iterations, rep.last_delta);
        assert!(err <= 1e-6, "max displacement from the true motion: {err} mm");
    }

    #[test]
    fn noisy_scan_converges_close_to_truth() {
        let target = synth::bracket();
        let truth = perturbation();
        let mut source = target.transformed(&truth.inverse());
        synth::add_noise(&mut source, 0.02, &mut Rng::new(3));
        let bvh = Bvh::build(&target);
        let rep = icp(&source, &bvh, &Transform::identity(), &IcpOptions::default()).unwrap();
        assert!(rep.converged);
        let err = rep.transform.max_displacement(&truth, &target.positions);
        assert!(err < 0.02, "max displacement {err}");
        assert!(rep.rms < 0.03, "rms {}", rep.rms);
    }

    #[test]
    fn plane_slides_without_failing() {
        // a plane on a plane: in-plane motion is undetermined, the solver must stay finite
        let target = synth::plane(Point3::origin(), Vec3::x(), Vec3::y(), 50.0, 50.0, 20, 20);
        let source = target.transformed(&Transform::translation(Vec3::new(0.0, 0.0, 0.5)));
        let rep = icp(&source, &Bvh::build(&target), &Transform::identity(), &IcpOptions::default()).unwrap();
        assert!(rep.transform.translation.z.abs() > 0.49 - 1e-6 && rep.rms < 1e-6);
    }

    #[test]
    fn empty_inputs_are_errors() {
        let bvh = Bvh::build(&synth::bracket());
        assert!(icp(&Mesh::default(), &bvh, &Transform::identity(), &IcpOptions::default()).is_err());
        assert!(icp(&synth::bracket(), &Bvh::default(), &Transform::identity(), &IcpOptions::default()).is_err());
    }
}
