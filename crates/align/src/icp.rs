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
use surfacecraft_geom::{Bvh, Mesh, Point3, Transform, Vec3};

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
    let pts = samples(source, opts.max_samples);
    if pts.len() < 6 {
        return Err(AlignError::NotEnough(format!("{} source points; at least 6 are needed", pts.len())));
    }
    if target.is_empty() {
        return Err(AlignError::NotEnough("the target has no triangles".into()));
    }
    let tol = if opts.tolerance.is_finite() && opts.tolerance > 0.0 { opts.tolerance } else { 1e-4 };
    let mut t = *init;
    let mut deltas = Vec::new();
    let mut converged = false;
    let mut rms = f64::INFINITY;
    let mut pairs = 0;
    let mut lambda = 0.0f64;
    for _ in 0..opts.max_iterations.max(1) {
        // correspondences
        let moved: Vec<Point3> = pts.iter().map(|p| t.apply(p)).collect();
        let hits: Vec<Option<(Point3, Vec3, f64)>> = surfacecraft_geom::par::map_slice(&moved, |p| {
            target
                .closest(p, f64::INFINITY)
                .and_then(|h| target.triangle_normal(h.triangle).map(|n| (h.point, n, h.distance)))
        });
        let mut dists: Vec<f64> = hits.iter().flatten().map(|h| h.2).collect();
        if dists.len() < 6 {
            return Err(AlignError::NotEnough("too few correspondences".into()));
        }
        dists.sort_by(f64::total_cmp);
        let median = dists.get(dists.len() / 2).copied().unwrap_or(0.0);
        let cut = opts.max_distance.filter(|d| d.is_finite() && *d > 0.0).unwrap_or((3.0 * median).max(10.0 * tol));
        // normal equations of r = n·(p − q) + (p × n)·w + n·t
        let mut ata = Matrix6::<f64>::zeros();
        let mut atb = Vector6::<f64>::zeros();
        let mut sq = 0.0;
        pairs = 0;
        // centre the rotation at the sample centroid for conditioning
        let c: Vec3 = moved.iter().map(|p| p.coords).sum::<Vec3>() / moved.len() as f64;
        for (p, h) in moved.iter().zip(&hits) {
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
        rms = (sq / pairs as f64).sqrt();
        // Levenberg damping only where the system is singular (sliding directions)
        let scale = ata.diagonal().max().max(1e-300);
        let sol = loop {
            let damped = ata + Matrix6::identity() * (lambda * scale);
            if let Some(ch) = damped.cholesky() {
                break Some(ch.solve(&(-atb)));
            }
            lambda = if lambda == 0.0 { 1e-12 } else { lambda * 10.0 };
            if lambda > 1.0 {
                break None;
            }
        };
        let Some(x) = sol else {
            return Err(AlignError::Degenerate("the point-to-plane system is singular".into()));
        };
        let w = Vec3::new(x[0], x[1], x[2]);
        let tr = Vec3::new(x[3], x[4], x[5]);
        // motion about the centroid c: p' = R(p − c) + c + tr
        let step = Transform::translation(c + tr).then_after(&rot(&w)).then_after(&Transform::translation(-c));
        let next = step.then_after(&t);
        let delta = next.max_displacement(&t, &pts);
        t = next;
        deltas.push(delta);
        if delta < tol {
            converged = true;
            break;
        }
    }
    Ok(IcpReport {
        transform: t,
        iterations: deltas.len(),
        converged,
        rms,
        last_delta: deltas.last().copied().unwrap_or(0.0),
        deltas,
        pairs,
    })
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
