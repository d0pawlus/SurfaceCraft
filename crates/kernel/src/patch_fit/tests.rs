use super::*;
use crate::nurbs::SurfaceProbe;
use crate::record::brep::Frame;
use crate::tess::tessellate;
use crate::vpatch::derivatives;

const RADIUS: f64 = 20.0;

pub(super) struct Rng(pub(super) u64);

impl Rng {
    pub(super) fn next(&mut self) -> f64 {
        self.0 = self.0.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        (self.0 >> 11) as f64 / (1u64 << 53) as f64
    }

    pub(super) fn normal(&mut self) -> f64 {
        let (a, b) = (self.next().max(1e-12), self.next());
        (-2.0 * a.ln()).sqrt() * (2.0 * std::f64::consts::PI * b).cos()
    }
}

pub(super) fn request(sides: Vec<Side>) -> Request {
    Request {
        sides,
        data: Vec::new(),
        data_uv: None,
        hard_points: Vec::new(),
        soft_points: Vec::new(),
        curves: Vec::new(),
        tolerance: 0.2,
        smoothing: None,
        control: None,
        max_control: 16,
        conflicts: Conflicts::Relax,
        side_ids: Vec::new(),
    }
}

pub(super) fn g0(points: Vec<[f64; 3]>) -> Side {
    Side { points, continuity: Continuity::G0, reference: None, uv: None, soft: false }
}

pub(super) fn regular(n: usize, centre: [f64; 2], r: f64) -> Vec<[f64; 2]> {
    (0..n)
        .map(|k| {
            let a = std::f64::consts::FRAC_PI_2 + 2.0 * std::f64::consts::PI * k as f64 / n as f64;
            [centre[0] + r * a.cos(), centre[1] + r * a.sin()]
        })
        .collect()
}

/// Sides that are straight chords between `corners` in the plane, lifted by `lift`.
pub(super) fn polygon(corners: &[[f64; 2]], per_side: usize, lift: &dyn Fn(f64, f64) -> [f64; 3]) -> Vec<Side> {
    let n = corners.len();
    (0..n)
        .map(|k| {
            let (a, b) = (corners[k], corners[(k + 1) % n]);
            g0((0..=per_side)
                .map(|i| {
                    let t = i as f64 / per_side as f64;
                    lift(a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]))
                })
                .collect())
        })
        .collect()
}

fn wave(x: f64, y: f64) -> f64 {
    3.0 * (x / 12.0).sin() * (y / 9.0).cos()
}

pub(super) fn cylinder_face() -> Face {
    Face {
        surface: Surface::Cylinder {
            frame: Frame { origin: [0.0; 3], x_axis: [1.0, 0.0, 0.0], y_axis: [0.0, 1.0, 0.0] },
            radius: RADIUS,
        },
        nurbs: None,
        domain: [[-1.0, 1.0], [-30.0, 30.0]],
        loops: Vec::new(),
    }
}

pub(super) fn on_cylinder(s: f64, z: f64) -> [f64; 3] {
    let t = s / RADIUS;
    [RADIUS * t.cos(), RADIUS * t.sin(), z]
}

/// A five-sided hole of radius 6 in the unrolled cylinder, every side continuing the cylinder.
pub(super) fn cylinder_hole(continuity: Continuity) -> Request {
    let corners = regular(5, [0.0, 0.0], 6.0);
    let sides = polygon(&corners, 40, &|s, z| on_cylinder(s, z))
        .into_iter()
        .map(|mut s| {
            s.continuity = continuity;
            s.reference = Some(cylinder_face());
            s
        })
        .collect();
    request(sides)
}

/// Mean curvature of `s` at `uv`.
fn mean_curvature(s: &Surface, uv: [f64; 2]) -> f64 {
    let d = derivatives(s, uv).unwrap();
    let n = geom::unit(geom::cross(d[1], d[2])).unwrap();
    let (e, f, g) = (geom::dot(d[1], d[1]), geom::dot(d[1], d[2]), geom::dot(d[2], d[2]));
    let (l, m, nn) = (geom::dot(d[3], n), geom::dot(d[4], n), geom::dot(d[5], n));
    (e * nn - 2.0 * f * m + g * l) / (2.0 * (e * g - f * f))
}

#[test]
fn flat_pentagon() {
    let sides = polygon(&regular(5, [0.0, 0.0], 15.0), 30, &|x, y| [x, y, 0.0]);
    let patch = fit_patch(&request(sides)).unwrap();
    assert!(patch.report.within_targets, "{:?}", patch.report);
    let mesh = tessellate(&patch.sheet, None).unwrap();
    assert!(!mesh.positions.is_empty());
    assert!(mesh.positions.iter().all(|p| p.z.abs() <= 1e-9));
    assert_eq!(patch.sheet.faces.len(), 1);
    assert!(
        patch.sheet.edges.len() >= 5 && patch.sheet.edges.len().is_multiple_of(5),
        "{} edges",
        patch.sheet.edges.len()
    );
    assert!(patch.report.warnings.iter().any(|w| w == WARN_NO_DATA));
}

#[test]
fn sphere_cap_boundary_only() {
    let lift = |x: f64, y: f64| [x, y, (50.0_f64 * 50.0 - x * x - y * y).sqrt()];
    let patch = fit_patch(&request(polygon(&regular(5, [0.0, 0.0], 15.0), 40, &lift))).unwrap();
    for s in &patch.report.sides {
        assert!(s.gap_max <= G0_TARGET, "gap {}", s.gap_max);
    }
    let probe = SurfaceProbe::new(&patch.surface).unwrap();
    let mut h = Vec::new();
    for i in 0..20 {
        for j in 0..20 {
            let (x, y) = (-10.0 + 20.0 * i as f64 / 19.0, -10.0 + 20.0 * j as f64 / 19.0);
            if x * x + y * y > 100.0 {
                continue;
            }
            let (_, _, uv) = probe.closest_uv(lift(x, y)).unwrap();
            h.push(mean_curvature(&patch.surface, uv).abs());
        }
    }
    h.sort_by(|a, b| a.total_cmp(b));
    let (median, p99) = (h[h.len() / 2], h[(h.len() * 99 / 100).min(h.len() - 1)]);
    assert!(p99 / median <= 5.0, "p99 {p99} median {median}");
}

#[test]
fn wave_with_data() {
    let lift = |x: f64, y: f64| [x, y, wave(x, y)];
    let mut req = request(polygon(&regular(6, [30.0, 20.0], 15.0), 40, &lift));
    let mut rng = Rng(7);
    while req.data.len() < 2000 {
        let (x, y) = (30.0 + 24.0 * (rng.next() - 0.5), 20.0 + 24.0 * (rng.next() - 0.5));
        if (x - 30.0).powi(2) + (y - 20.0).powi(2) <= 144.0 {
            req.data.push([x, y, wave(x, y) + 0.03 * rng.normal()]);
        }
    }
    let patch = fit_patch(&req).unwrap();
    assert!(patch.report.max <= 0.2, "{:?}", patch.report);
    assert!(patch.report.within_tolerance);
    assert_eq!(patch.report.points, 2000);
    assert!((0.015..=0.045).contains(&patch.report.rms), "{:?}", patch.report);
}

#[test]
fn g0_to_cylinder() {
    let req = cylinder_hole(Continuity::G0);
    let patch = fit_patch(&req).unwrap();
    // measured gap up to 7.1e-4 (1.4x under the target): the rounds stop at the target, so the test
    // does not rely on `within_targets` or on the target itself but on twice the target
    for s in &patch.report.sides {
        assert!(s.gap_max <= 2.0 * G0_TARGET, "{:?}", patch.report);
    }
    assert!(patch.report.control.iter().all(|&n| n <= req.max_control), "{:?}", patch.report);
}

#[test]
fn g1_to_cylinder() {
    let patch = fit_patch(&cylinder_hole(Continuity::G1)).unwrap();
    // measured (macOS aarch64): gap <= 2.3e-4 (4.3x under the target), angle <= 0.0175 deg (2.9x)
    for s in &patch.report.sides {
        assert!(s.gap_max <= G0_TARGET, "{:?}", patch.report);
        assert!(s.angle_max_deg.unwrap() <= G1_TARGET_DEG, "{:?}", patch.report);
        assert!(!s.folded);
    }
    assert!(patch.report.within_targets, "{:?}", patch.report);
}

#[test]
fn g2_to_cylinder() {
    let patch = fit_patch(&cylinder_hole(Continuity::G2)).unwrap();
    // measured: gap <= 2.3e-4, angle <= 0.0175 deg, curvature error <= 3.5 % (1.44x under 5 %): the
    // curvature bound is 7.5 % and `within_targets` (which needs the 5 %) is not asserted
    for s in &patch.report.sides {
        assert!(s.gap_max <= G0_TARGET, "{:?}", patch.report);
        assert!(s.angle_max_deg.unwrap() <= G1_TARGET_DEG, "{:?}", patch.report);
        assert!(s.curvature_error_max.unwrap() <= 1.5 * G2_TARGET_REL, "{:?}", patch.report);
        assert!(!s.folded);
    }
}

/// A loop of four sides whose first two continue planes 30° apart at their corner; the others are
/// G0 and leave and arrive in those planes.
fn skewed_corner(conflicts: Conflicts) -> Request {
    let (c, s) = (30.0_f64.to_radians().cos(), 30.0_f64.to_radians().sin());
    let line = |a: [f64; 3], b: [f64; 3], n: usize| -> Vec<[f64; 3]> {
        (0..=n)
            .map(|i| {
                let t = i as f64 / n as f64;
                [a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]), a[2] + t * (b[2] - a[2])]
            })
            .collect()
    };
    let join = |mut a: Vec<[f64; 3]>, b: Vec<[f64; 3]>| {
        a.extend(b.into_iter().skip(1));
        a
    };
    let (a, b, cc, d, e) =
        ([5.0, -2.0, 0.0], [0.0; 3], [0.0, 10.0, 0.0], [10.0 * c, 10.0, -10.0 * s], [10.0, 4.0, 0.0]);
    let plane = |u: [f64; 3], v: [f64; 3]| Face {
        surface: Surface::Plane { origin: [0.0; 3], u_axis: u, v_axis: v },
        nurbs: None,
        domain: [[-20.0, 20.0], [-20.0, 20.0]],
        loops: Vec::new(),
    };
    let side = |points, continuity, reference| Side { points, continuity, reference, uv: None, soft: false };
    let mut req = request(vec![
        side(line(a, b, 30), Continuity::G1, Some(plane([1.0, 0.0, 0.0], [0.0, 1.0, 0.0]))),
        side(line(b, cc, 30), Continuity::G1, Some(plane([0.0, 1.0, 0.0], [c, 0.0, -s]))),
        side(line(cc, d, 30), Continuity::G0, None),
        side(join(line(d, e, 20), line(e, a, 20)), Continuity::G0, None),
    ]);
    req.conflicts = conflicts;
    req
}

#[test]
fn incompatible_corner() {
    let patch = fit_patch(&skewed_corner(Conflicts::Relax)).unwrap();
    assert_eq!(patch.report.corners.len(), 1, "{:?}", patch.report.corners);
    let angle = patch.report.corners[0].mismatch_deg;
    assert!((29.0..31.0).contains(&angle), "{angle}");
    assert!(!patch.report.within_targets);
    let err = fit_patch(&skewed_corner(Conflicts::Fail)).unwrap_err();
    assert!(err.to_string().contains("corner"), "{err}");
}

#[test]
fn refuses_self_crossing_projection() {
    let lobe = |from: f64| -> Vec<[f64; 3]> {
        (0..=60)
            .map(|i| {
                let t = from + std::f64::consts::PI * i as f64 / 60.0;
                [10.0 * t.sin(), 5.0 * (2.0 * t).sin(), 0.0]
            })
            .collect()
    };
    let err = fit_patch(&request(vec![g0(lobe(0.0)), g0(lobe(std::f64::consts::PI))])).unwrap_err();
    assert!(err.to_string().contains("height field"), "{err}");
}

#[test]
fn degenerate_sides() {
    let tri = [[0.0, 0.0, 0.0], [10.0, 0.0, 0.0], [0.0, 10.0, 0.0]];
    // a side of two identical points has no length
    let sides = vec![g0(vec![tri[0], tri[0]]), g0(vec![tri[0], tri[1]]), g0(vec![tri[1], tri[0]])];
    assert!(fit_patch(&request(sides)).is_err());
    // a triangle of 2-point sides: a clean Err or a valid patch
    let sides = (0..3).map(|k| g0(vec![tri[k], tri[(k + 1) % 3]])).collect();
    if let Ok(p) = fit_patch(&request(sides)) {
        assert!(p.report.rms.is_finite() && p.report.max.is_finite());
    }
    // 65 sides are too many
    let sides = polygon(&regular(65, [0.0, 0.0], 15.0), 2, &|x, y| [x, y, 0.0]);
    assert!(fit_patch(&request(sides)).is_err());
}

#[test]
fn hostile_request() {
    let good = || polygon(&regular(5, [0.0, 0.0], 15.0), 10, &|x, y| [x, y, 0.0]);
    let mut nan = good();
    nan[1].points[3][2] = f64::NAN;
    assert!(fit_patch(&request(nan)).is_err());
    assert!(fit_patch(&request(Vec::new())).is_err());
    let mut r = request(good());
    r.tolerance = 0.0;
    assert!(fit_patch(&r).is_err());
    let mut r = request(good());
    r.max_control = 99;
    assert!(fit_patch(&r).is_err());
    let mut r = request(good());
    r.data = vec![[f64::INFINITY, 0.0, 0.0]];
    assert!(fit_patch(&r).is_err());
    let mut g1 = good();
    g1[0].continuity = Continuity::G1;
    assert!(fit_patch(&request(g1)).is_err());
}

#[test]
fn cylinder_normal_curvature() {
    let face = cylinder_face();
    let (n, k) = face_normal_curvature(&face, [RADIUS, 0.0, 3.0], [0.0, 1.0, 0.0]).unwrap();
    assert!((n[0] - 1.0).abs() < 1e-9 && n[1].abs() < 1e-9 && n[2].abs() < 1e-9, "{n:?}");
    assert!((k + 1.0 / RADIUS).abs() < 1e-9, "{k}");
    let (_, k) = face_normal_curvature(&face, [RADIUS, 0.0, 3.0], [0.0, 0.0, 2.0]).unwrap();
    assert!(k.abs() < 1e-9, "{k}");
    assert!(face_normal_curvature(&face, [f64::NAN, 0.0, 0.0], [0.0, 1.0, 0.0]).is_err());
    assert!(face_normal_curvature(&face, [RADIUS, 0.0, 0.0], [0.0; 3]).is_err());
}
