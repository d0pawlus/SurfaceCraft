//! Measurement (not a gate): why does `fit_patch` give an rms of 2 to 4 times the noise on a planar
//! face with noisy scan data?
//!
//! A regular pentagon (or triangle) in the plane z = 0 with five straight G0 sides and about 1500
//! points of data (z = noise) is fitted by a baseline request and by variants that each change ONE
//! thing: no noise, a fixed control net, an explicit smoothing, supplied parameters (exact and
//! scaled), data up to the edge or outside it, sides that disagree with the data by 0.05 mm, noisy
//! sides, a triangle, no data. For every case one line gives the report, a second line the rms of the
//! data distance to the returned surface binned by the data's in-plane distance to the outline (does
//! the error sit at the boundary or everywhere?), and a third line the distance of the surface from
//! the TRUE plane at the data's (x, y) (bias against noise). Nothing is asserted; every kernel error
//! is printed as `FAILED: <error>` and the run goes on. Run it with
//! `cargo test -p surfacecraft-kernel --test patch_diag -- --ignored --nocapture`.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::f64::consts::{FRAC_PI_2, PI};
use surfacecraft_kernel::KernelError;
use surfacecraft_kernel::nurbs::SurfaceProbe;
use surfacecraft_kernel::patch_fit::{Conflicts, Continuity, Request, Side, fit_patch};
use surfacecraft_kernel::record::brep::Surface;

/// Points of every side polyline.
const SIDE_POINTS: usize = 60;
/// Noise of the scan, mm.
const SIGMA: f64 = 0.03;

/// A small LCG with Box-Muller normals; one stream per purpose so that the variants share their data.
struct Rng(u64);

impl Rng {
    fn new(seed: u64) -> Self {
        let mut r = Self(seed.wrapping_mul(0x9E37_79B9_7F4A_7C15) ^ 0xD1B5_4A32_D192_ED03);
        for _ in 0..8 {
            r.uniform();
        }
        r
    }

    fn uniform(&mut self) -> f64 {
        self.0 = self.0.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        (self.0 >> 11) as f64 / (1u64 << 53) as f64
    }

    fn normal(&mut self) -> f64 {
        let (a, b) = (self.uniform().max(1e-12), self.uniform());
        (-2.0 * a.ln()).sqrt() * (2.0 * PI * b).cos()
    }
}

#[derive(Clone, Copy, PartialEq)]
enum UvMode {
    /// No parameters given: the driver fits a plane.
    Absent,
    /// uv = (x, y).
    Exact,
    /// uv = (0.37 x + 5, 0.37 y - 3).
    Scaled,
}

fn map_uv(mode: UvMode, x: f64, y: f64) -> [f64; 2] {
    match mode {
        UvMode::Scaled => [0.37 * x + 5.0, 0.37 * y - 3.0],
        _ => [x, y],
    }
}

/// What one case changes against the baseline.
#[derive(Clone)]
struct Setup {
    sides: usize,
    radius: f64,
    /// About this many data points inside the polygon.
    data: usize,
    /// Data stay this far (mm) inside the outline.
    shrink: f64,
    /// Noise of the data in z.
    sigma: f64,
    /// Extra data points in a 1 mm band outside the polygon.
    outside: usize,
    /// Amplitude of the lift of the side points along the side, `lift * sin(pi t)`.
    lift: f64,
    /// Noise in z at every side point except the corners.
    side_sigma: f64,
    uv: UvMode,
    control: Option<[usize; 2]>,
    smoothing: Option<f64>,
    /// The data are given to the fit (the residual lines still use them).
    give_data: bool,
}

impl Setup {
    fn baseline() -> Self {
        Self {
            sides: 5,
            radius: 20.0,
            data: 1500,
            shrink: 0.5,
            sigma: SIGMA,
            outside: 0,
            lift: 0.0,
            side_sigma: 0.0,
            uv: UvMode::Absent,
            control: None,
            smoothing: None,
            give_data: true,
        }
    }

    fn with(&self, f: impl FnOnce(&mut Setup)) -> Self {
        let mut s = self.clone();
        f(&mut s);
        s
    }
}

/// The request of a case, the points the residuals are measured on, and the outline in the plane.
struct Built {
    req: Request,
    eval: Vec<[f64; 3]>,
    outline: Vec<[f64; 2]>,
}

fn polygon(n: usize, r: f64) -> Vec<[f64; 2]> {
    (0..n)
        .map(|k| {
            let a = FRAC_PI_2 + 2.0 * PI * k as f64 / n as f64;
            [r * a.cos(), r * a.sin()]
        })
        .collect()
}

fn seg_dist(p: [f64; 2], a: [f64; 2], b: [f64; 2]) -> f64 {
    let (dx, dy) = (b[0] - a[0], b[1] - a[1]);
    let l2 = dx * dx + dy * dy;
    let t = if l2 > 0.0 { (((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / l2).clamp(0.0, 1.0) } else { 0.0 };
    (p[0] - a[0] - t * dx).hypot(p[1] - a[1] - t * dy)
}

/// Distance of `p` to the polygon's boundary (unsigned).
fn boundary_distance(p: [f64; 2], poly: &[[f64; 2]]) -> f64 {
    let n = poly.len();
    (0..n).map(|k| seg_dist(p, poly[k], poly[(k + 1) % n])).fold(f64::INFINITY, f64::min)
}

/// Is `p` at least `margin` inside every edge of the convex counter-clockwise polygon?
fn inside_by(p: [f64; 2], poly: &[[f64; 2]], margin: f64) -> bool {
    let n = poly.len();
    (0..n).all(|k| {
        let (a, b) = (poly[k], poly[(k + 1) % n]);
        let (dx, dy) = (b[0] - a[0], b[1] - a[1]);
        (dx * (p[1] - a[1]) - dy * (p[0] - a[0])) / dx.hypot(dy) >= margin
    })
}

fn build(s: &Setup) -> Built {
    let poly = polygon(s.sides, s.radius);
    let n = poly.len();

    // data xy: a jittered grid inside the polygon, shrunk by `shrink`
    let apothem = s.radius * (PI / n as f64).cos();
    let area = 0.5 * n as f64 * s.radius * s.radius * (2.0 * PI / n as f64).sin();
    let area_eff = area * ((apothem - s.shrink) / apothem).powi(2);
    let h = (area_eff / s.data as f64).sqrt();
    let cells = (2.0 * s.radius / h).ceil() as usize + 1;
    let mut jitter = Rng::new(1);
    let mut xy: Vec<[f64; 2]> = Vec::new();
    for i in 0..cells {
        for j in 0..cells {
            let x = -s.radius + (i as f64 + jitter.uniform()) * h;
            let y = -s.radius + (j as f64 + jitter.uniform()) * h;
            if inside_by([x, y], &poly, s.shrink) {
                xy.push([x, y]);
            }
        }
    }
    // a band of data outside the polygon
    let mut band = Rng::new(4);
    let mut got = 0usize;
    let mut draws = 0usize;
    while got < s.outside && draws < 2_000_000 {
        draws += 1;
        let reach = s.radius + 1.5;
        let p = [(2.0 * band.uniform() - 1.0) * reach, (2.0 * band.uniform() - 1.0) * reach];
        if !inside_by(p, &poly, 0.0) && boundary_distance(p, &poly) <= 1.0 {
            xy.push(p);
            got += 1;
        }
    }
    let mut noise = Rng::new(2);
    let eval: Vec<[f64; 3]> = xy.iter().map(|p| [p[0], p[1], s.sigma * noise.normal()]).collect();

    // sides: straight polylines, optionally lifted or noisy in z; the corners stay exact
    let mut side_noise = Rng::new(3);
    let sides: Vec<Side> = (0..n)
        .map(|k| {
            let (a, b) = (poly[k], poly[(k + 1) % n]);
            let points: Vec<[f64; 3]> = (0..SIDE_POINTS)
                .map(|i| {
                    let t = i as f64 / (SIDE_POINTS - 1) as f64;
                    let (x, y) = (a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]));
                    let end = i == 0 || i + 1 == SIDE_POINTS;
                    let mut z = 0.0;
                    if !end {
                        z += s.lift * (PI * t).sin() + s.side_sigma * side_noise.normal();
                    }
                    [x, y, z]
                })
                .collect();
            let uv = (s.uv != UvMode::Absent).then(|| points.iter().map(|p| map_uv(s.uv, p[0], p[1])).collect());
            Side { points, continuity: Continuity::G0, reference: None, uv, soft: false }
        })
        .collect();

    let (data, data_uv) = if s.give_data {
        let uv = (s.uv != UvMode::Absent).then(|| eval.iter().map(|p| map_uv(s.uv, p[0], p[1])).collect());
        (eval.clone(), uv)
    } else {
        (Vec::new(), None)
    };
    let req = Request {
        sides,
        data,
        data_uv,
        hard_points: Vec::new(),
        soft_points: Vec::new(),
        curves: Vec::new(),
        tolerance: 0.2,
        smoothing: s.smoothing,
        control: s.control,
        max_control: 16,
        conflicts: Conflicts::Relax,
    };
    Built { req, eval, outline: poly }
}

/// `rms (n)` of the residuals in three bins of the distance to the outline.
fn binned(pairs: &[(f64, f64)]) -> String {
    let mut sum = [0.0_f64; 3];
    let mut count = [0usize; 3];
    for (edge, d) in pairs {
        let k = if *edge < 1.0 {
            0
        } else if *edge < 3.0 {
            1
        } else {
            2
        };
        sum[k] += d * d;
        count[k] += 1;
    }
    let rms = |k: usize| if count[k] == 0 { 0.0 } else { (sum[k] / count[k] as f64).sqrt() };
    format!(
        "0-1 mm: rms {:.4} ({}) | 1-3 mm: rms {:.4} ({}) | >3 mm: rms {:.4} ({})",
        rms(0),
        count[0],
        rms(1),
        count[1],
        rms(2),
        count[2]
    )
}

/// The two residual lines: the data against the surface, and the true plane against the surface.
fn residual_lines(surface: &Surface, b: &Built) -> Result<(String, String), KernelError> {
    let probe = SurfaceProbe::new(surface)?;
    let mut data = Vec::with_capacity(b.eval.len());
    let mut truth = Vec::with_capacity(b.eval.len());
    for p in &b.eval {
        let edge = boundary_distance([p[0], p[1]], &b.outline);
        let (d, _) = probe.closest(*p)?;
        data.push((edge, d));
        let (t, _) = probe.closest([p[0], p[1], 0.0])?;
        truth.push((edge, t));
    }
    let first = format!("      residual by distance to the outline: {}", binned(&data));
    let rms = (truth.iter().map(|(_, t)| t * t).sum::<f64>() / truth.len().max(1) as f64).sqrt();
    let max = truth.iter().map(|(_, t)| *t).fold(0.0_f64, f64::max);
    let second = format!(
        "      patch against the true plane z = 0 at the data xy: rms {rms:.4} max {max:.4} | {}",
        binned(&truth)
    );
    Ok((first, second))
}

fn run(name: &str, s: &Setup) {
    let b = build(s);
    let patch = match fit_patch(&b.req) {
        Ok(p) => p,
        Err(e) => {
            eprintln!("CASE {name}: FAILED {e}");
            return;
        }
    };
    let r = &patch.report;
    let gap = r.sides.iter().map(|x| x.gap_max).fold(0.0_f64, f64::max);
    eprintln!(
        "CASE {name}: control [{}, {}] rounds {} | rms {:.4} max {:.4} p99 {:.4} over {} points | side gap max {:.5} | warnings {:?}",
        r.control[0], r.control[1], r.rounds, r.rms, r.max, r.p99, r.points, gap, r.warnings
    );
    match residual_lines(&patch.surface, &b) {
        Ok((a, t)) => {
            eprintln!("{a}");
            eprintln!("{t}");
        }
        Err(e) => eprintln!("      residual lines FAILED {e}"),
    }
}

#[test]
#[ignore = "measurement; run with --ignored --nocapture"]
fn diagnose_noisy_plane() {
    let base = Setup::baseline();
    let mut cases: Vec<(String, Setup)> = Vec::new();
    cases.push(("baseline".into(), base.clone()));
    cases.push(("no_noise".into(), base.with(|s| s.sigma = 0.0)));
    for n in [8usize, 12] {
        cases.push((format!("fixed_net_{n}"), base.with(|s| s.control = Some([n, n]))));
    }
    for (label, value) in [("1e-1", 1e-1), ("1e-2", 1e-2), ("1e-4", 1e-4), ("1e-6", 1e-6)] {
        cases.push((format!("smoothing_{label}"), base.with(|s| s.smoothing = Some(value))));
    }
    cases.push(("uv_given".into(), base.with(|s| s.uv = UvMode::Exact)));
    cases.push(("uv_given_scaled".into(), base.with(|s| s.uv = UvMode::Scaled)));
    cases.push(("data_to_the_edge".into(), base.with(|s| s.shrink = 0.0)));
    cases.push(("data_outside".into(), base.with(|s| s.outside = 150)));
    cases.push(("sides_off_plane_0.05".into(), base.with(|s| s.lift = 0.05)));
    cases.push(("sides_noisy".into(), base.with(|s| s.side_sigma = SIGMA)));
    cases.push((
        "triangle".into(),
        base.with(|s| {
            s.sides = 3;
            s.radius = 12.0;
            s.data = 400;
        }),
    ));
    cases.push(("no_data".into(), base.with(|s| s.give_data = false)));

    for (name, setup) in &cases {
        run(name, setup);
    }
}
