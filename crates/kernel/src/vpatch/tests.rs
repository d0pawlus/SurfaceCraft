use super::*;

fn wave(x: f64, y: f64) -> f64 {
    3.0 * (x / 12.0).sin() * (y / 9.0).cos()
}

fn net(s: &Surface) -> Vec<Vec<[f64; 3]>> {
    match s {
        Surface::Spline { points, .. } => points.clone(),
        _ => panic!("expected a spline"),
    }
}

/// 8 × 8 bicubic surface over [0, 1]² whose control net is the wave on a 60 × 40 grid.
fn known_surface() -> Surface {
    let points: Vec<Vec<[f64; 3]>> = (0..8)
        .map(|i| {
            (0..8)
                .map(|j| {
                    let (x, y) = (60.0 * i as f64 / 7.0, 40.0 * j as f64 / 7.0);
                    [x, y, wave(x, y)]
                })
                .collect()
        })
        .collect();
    Surface::Spline { u_knots: clamped_uniform(8, 0.0, 1.0), v_knots: clamped_uniform(8, 0.0, 1.0), points }
}

fn sample(s: &Surface, uv: [f64; 2]) -> [f64; 3] {
    derivatives(s, uv).unwrap()[0]
}

const UNIT: [[f64; 2]; 2] = [[0.0, 1.0], [0.0, 1.0]];

fn plane(x: f64, y: f64) -> f64 {
    0.2 * x - 0.1 * y + 3.0
}

fn on_plane(uv: [f64; 2]) -> [f64; 3] {
    let (x, y) = (50.0 * uv[0], 30.0 * uv[1]);
    [x, y, plane(x, y)]
}

const PENTAGON: [[f64; 2]; 5] = [[0.1, 0.15], [0.85, 0.1], [0.95, 0.6], [0.5, 0.92], [0.08, 0.55]];

fn edge_point(e: usize, t: f64) -> [f64; 2] {
    let (a, b) = (PENTAGON[e], PENTAGON[(e + 1) % 5]);
    [a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t]
}

fn pentagon_rows() -> Vec<Row> {
    let mut rows = Vec::new();
    for e in 0..5 {
        for k in 0..8 {
            let uv = edge_point(e, k as f64 / 8.0);
            rows.push(Row::Position { uv, at: on_plane(uv) });
        }
    }
    rows
}

fn plane_problem(hard: Vec<Row>) -> Problem {
    Problem {
        domain: UNIT,
        control: [8, 8],
        hard,
        soft: Vec::new(),
        data: Vec::new(),
        outline: Vec::new(),
        smoothing: 1.0,
    }
}

#[test]
fn recovers_known_surface() {
    let known = known_surface();
    let mut data = Vec::new();
    for i in 0..40 {
        for j in 0..40 {
            let uv = [i as f64 / 39.0, j as f64 / 39.0];
            data.push(DataPoint { uv, at: sample(&known, uv), weight: 1.0 });
        }
    }
    let mut hard = Vec::new();
    for side in 0..4 {
        for k in 0..12 {
            let t = k as f64 / 12.0;
            let uv = match side {
                0 => [t, 0.0],
                1 => [1.0, t],
                2 => [1.0 - t, 1.0],
                _ => [0.0, 1.0 - t],
            };
            hard.push(Row::Position { uv, at: sample(&known, uv) });
        }
    }
    let p =
        Problem { domain: UNIT, control: [8, 8], hard, soft: Vec::new(), data, outline: Vec::new(), smoothing: 1e-9 };
    let sol = solve(&p).unwrap();
    let (got, want) = (net(&sol.surface), net(&known));
    for i in 0..8 {
        for j in 0..8 {
            for c in 0..3 {
                let d = (got[i][j][c] - want[i][j][c]).abs();
                assert!(d <= 1e-6 * 72.1, "control point {i},{j},{c} off by {d}");
            }
        }
    }
}

#[test]
fn boundary_only_plane() {
    let sol = solve(&plane_problem(pentagon_rows())).unwrap();
    let mut worst = 0.0_f64;
    for i in 0..30 {
        for j in 0..30 {
            let s = sample(&sol.surface, [i as f64 / 29.0, j as f64 / 29.0]);
            worst = worst.max((s[2] - plane(s[0], s[1])).abs());
        }
    }
    assert!(worst <= 1e-9, "max deviation from the plane {worst}");
}

#[test]
fn tangent_rows_hold() {
    let mut hard = pentagon_rows();
    let normal = [0.0, 0.6, 0.8];
    let mut at = Vec::new();
    for k in 1..=6 {
        let uv = edge_point(0, k as f64 / 8.0);
        at.push(uv);
        hard.push(Row::Tangent { uv, dir: [0.0, 1.0], normal });
    }
    let sol = solve(&plane_problem(hard)).unwrap();
    for uv in at {
        let d = derivatives(&sol.surface, uv).unwrap();
        let dot = d[2][0] * normal[0] + d[2][1] * normal[1] + d[2][2] * normal[2];
        assert!(dot.abs() <= 1e-9, "S_v · n = {dot} at {uv:?}");
    }
}

#[test]
fn curvature_rows_hold() {
    let mut hard = pentagon_rows();
    let uv = [0.5, 0.5];
    hard.push(Row::Curvature { uv, dir: [1.0, 0.0], normal: [0.0, 0.0, 1.0], value: 0.02 });
    let sol = solve(&plane_problem(hard)).unwrap();
    let d = derivatives(&sol.surface, uv).unwrap();
    assert!((d[3][2] - 0.02).abs() <= 1e-9, "S_uu · n = {}", d[3][2]);
}

#[test]
fn redundant_rows_are_dropped() {
    let once = pentagon_rows();
    let mut twice = once.clone();
    twice.extend(once.iter().cloned());
    let a = solve(&plane_problem(once.clone())).unwrap();
    let b = solve(&plane_problem(twice)).unwrap();
    assert_eq!(b.dropped, a.dropped + 3 * once.len(), "dropped rows");
    let (pa, pb) = (net(&a.surface), net(&b.surface));
    for i in 0..8 {
        for j in 0..8 {
            for c in 0..3 {
                assert!((pa[i][j][c] - pb[i][j][c]).abs() <= 1e-9, "control point {i},{j},{c} differs");
            }
        }
    }
}

struct Lcg(u64);

impl Lcg {
    fn next_unit(&mut self) -> f64 {
        self.0 = self.0.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        (self.0 >> 11) as f64 / (1u64 << 53) as f64
    }
}

#[test]
fn noise_stays_smooth() {
    let mut rng = Lcg(7);
    // uniform noise of standard deviation 0.03
    let half_width = 0.03 * 3.0_f64.sqrt();
    let mut data = Vec::new();
    for i in 0..40 {
        for j in 0..40 {
            let uv = [i as f64 / 39.0, j as f64 / 39.0];
            let (x, y) = (60.0 * uv[0], 40.0 * uv[1]);
            let noise = (2.0 * rng.next_unit() - 1.0) * half_width;
            data.push(DataPoint { uv, at: [x, y, wave(x, y) + noise], weight: 1.0 });
        }
    }
    let p = Problem {
        domain: UNIT,
        control: [10, 10],
        hard: Vec::new(),
        soft: Vec::new(),
        data: data.clone(),
        outline: Vec::new(),
        smoothing: 1e-3,
    };
    let sol = solve(&p).unwrap();
    let mut sum = 0.0;
    for d in &data {
        let s = sample(&sol.surface, d.uv);
        sum += (0..3).map(|c| (d.at[c] - s[c]).powi(2)).sum::<f64>();
    }
    let rms = (sum / data.len() as f64).sqrt();
    assert!((0.015..=0.045).contains(&rms), "residual rms {rms}");
}

fn data_rms(s: &Surface, data: &[DataPoint]) -> f64 {
    let mut sum = 0.0;
    for d in data {
        let p = sample(s, d.uv);
        sum += (0..3).map(|c| (d.at[c] - p[c]).powi(2)).sum::<f64>();
    }
    (sum / data.len() as f64).sqrt()
}

#[test]
fn soft_rows_do_not_stiffen_the_fit() {
    let mut rng = Lcg(11);
    // uniform noise of standard deviation 0.03 on a 40 x 24 region of the wave, uv = (x, y)
    let half_width = 0.03 * 3.0_f64.sqrt();
    let mut data = Vec::new();
    for _ in 0..2000 {
        let (x, y) = (40.0 * rng.next_unit(), 24.0 * rng.next_unit());
        let noise = (2.0 * rng.next_unit() - 1.0) * half_width;
        data.push(DataPoint { uv: [x, y], at: [x, y, wave(x, y) + noise], weight: 1.0 });
    }
    // 1000 exact positions along the boundary of the domain, perimeter 128
    let mut soft = Vec::new();
    for k in 0..1000 {
        let s = 128.0 * k as f64 / 1000.0;
        let (x, y) = if s < 40.0 {
            (s, 0.0)
        } else if s < 64.0 {
            (40.0, s - 40.0)
        } else if s < 104.0 {
            (104.0 - s, 24.0)
        } else {
            (0.0, 128.0 - s)
        };
        soft.push((Row::Position { uv: [x, y], at: [x, y, wave(x, y)] }, 1e4));
    }
    let free = Problem {
        domain: [[0.0, 40.0], [0.0, 24.0]],
        control: [12, 12],
        hard: Vec::new(),
        soft: Vec::new(),
        data: data.clone(),
        outline: Vec::new(),
        smoothing: 1e-3,
    };
    let pinned = Problem { soft, ..free.clone() };
    let rms_free = data_rms(&solve(&free).unwrap().surface, &data);
    let rms_pinned = data_rms(&solve(&pinned).unwrap().surface, &data);
    assert!((0.015..=0.045).contains(&rms_free), "residual rms without soft rows {rms_free}");
    assert!((0.015..=0.045).contains(&rms_pinned), "residual rms with soft rows {rms_pinned}");
    assert!((rms_free - rms_pinned).abs() < 0.003, "rms differs: {rms_free} vs {rms_pinned}");
}

#[test]
fn empty_area_is_filled_by_energy() {
    let mut data = Vec::new();
    for i in 0..20 {
        for j in 0..20 {
            let uv = [0.5 * i as f64 / 19.0, 0.5 * j as f64 / 19.0];
            let (x, y) = (60.0 * uv[0], 40.0 * uv[1]);
            data.push(DataPoint { uv, at: [x, y, wave(x, y)], weight: 1.0 });
        }
    }
    // bounding box diagonal of the data
    let (mut lo, mut hi) = ([f64::INFINITY; 3], [f64::NEG_INFINITY; 3]);
    for d in &data {
        for c in 0..3 {
            lo[c] = lo[c].min(d.at[c]);
            hi[c] = hi[c].max(d.at[c]);
        }
    }
    let diag = (0..3).map(|c| (hi[c] - lo[c]).powi(2)).sum::<f64>().sqrt();
    let p = Problem {
        domain: UNIT,
        control: [8, 8],
        hard: Vec::new(),
        soft: Vec::new(),
        data,
        outline: Vec::new(),
        smoothing: 1e-3,
    };
    let sol = solve(&p).unwrap();
    assert!(net(&sol.surface).iter().flatten().all(|c| c.iter().all(|v| v.is_finite())));
    let mut worst = 0.0_f64;
    for i in 0..30 {
        for j in 0..30 {
            let s = sample(&sol.surface, [i as f64 / 29.0, j as f64 / 29.0]);
            worst = worst.max((s[0] * s[0] + s[1] * s[1] + s[2] * s[2]).sqrt());
        }
    }
    assert!(worst <= 5.0 * diag, "max |S| {worst} against the data diagonal {diag}");
}

#[test]
fn nonconvex_outline_decouples_arms() {
    // a U: arm A at u < 0.3, arm B at u > 0.7, joined by the base at v < 0.3
    let outline = vec![[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.7, 1.0], [0.7, 0.3], [0.3, 0.3], [0.3, 1.0], [0.0, 1.0]];
    let arm = |u0: f64, f: &dyn Fn(f64, f64) -> f64| -> Vec<DataPoint> {
        let mut out = Vec::new();
        for i in 0..14 {
            for j in 0..20 {
                let uv = [u0 + 0.26 * i as f64 / 13.0, 0.05 + 0.9 * j as f64 / 19.0];
                out.push(DataPoint { uv, at: [60.0 * uv[0], 40.0 * uv[1], f(uv[0], uv[1])], weight: 1.0 });
            }
        }
        out
    };
    let fa = |u: f64, v: f64| 1.5 * (5.0 * u).sin() * (3.0 * v).cos();
    let fb = |u: f64, v: f64| 8.0 + 4.0 * v + 2.0 * (4.0 * u).sin();
    let a_only = arm(0.02, &fa);
    let mut both = a_only.clone();
    let b_data = arm(0.72, &fb);
    let amplitude = b_data.iter().map(|d| d.at[2].abs()).fold(0.0_f64, f64::max);
    both.extend(b_data);
    let make = |data: Vec<DataPoint>| Problem {
        domain: UNIT,
        control: [12, 12],
        hard: Vec::new(),
        soft: Vec::new(),
        data,
        outline: outline.clone(),
        smoothing: 1e-3,
    };
    let s1 = solve(&make(a_only)).unwrap();
    let s2 = solve(&make(both)).unwrap();
    let mut worst = 0.0_f64;
    for i in 0..8 {
        for j in 0..8 {
            let uv = [0.03 + 0.24 * i as f64 / 7.0, 0.5 + 0.45 * j as f64 / 7.0];
            worst = worst.max((sample(&s1.surface, uv)[2] - sample(&s2.surface, uv)[2]).abs());
        }
    }
    assert!(worst <= 0.1 * amplitude, "arm A moved by {worst}, arm B amplitude {amplitude}");
}

#[test]
fn hostile_input_is_refused() {
    let good = plane_problem(pentagon_rows());
    assert!(solve(&good).is_ok());
    let mut cases: Vec<Problem> = Vec::new();
    let mut p = good.clone();
    p.data.push(DataPoint { uv: [f64::NAN, 0.5], at: [0.0; 3], weight: 1.0 });
    cases.push(p);
    let mut p = good.clone();
    p.control = [3, 9];
    cases.push(p);
    let mut p = good.clone();
    p.control = [25, 4];
    cases.push(p);
    let mut p = good.clone();
    p.domain = [[1.0, 0.0], [0.0, 1.0]];
    cases.push(p);
    let mut p = good.clone();
    p.hard.push(Row::Tangent { uv: [0.5, 0.5], dir: [1.0, 0.0], normal: [0.0; 3] });
    cases.push(p);
    let mut p = good.clone();
    p.data.push(DataPoint { uv: [0.5, 0.5], at: [0.0; 3], weight: 0.0 });
    cases.push(p);
    let mut p = good.clone();
    p.hard.push(Row::Position { uv: [2.0, 0.5], at: [0.0; 3] });
    cases.push(p);
    let mut p = good.clone();
    p.outline = vec![[0.0, 0.0], [1.0, 1.0]];
    cases.push(p);
    let mut p = good.clone();
    p.hard.push(Row::Position { uv: [0.5, 0.5], at: [f64::INFINITY, 0.0, 0.0] });
    cases.push(p);
    for (i, c) in cases.iter().enumerate() {
        assert!(solve(c).is_err(), "case {i} must be refused");
    }
    let known = known_surface();
    assert!(derivatives(&known, [f64::NAN, 0.5]).is_err());
}

/// The wave problem with soft tangent and curvature rows, in the units of `k` (1 or 10 times larger):
/// positions and parameters scale with `k`, a curvature value with `1 / k`, nothing else changes.
fn scaled_problem(k: f64) -> Problem {
    let place = |uv: [f64; 2]| [k * 60.0 * uv[0], k * 40.0 * uv[1], k * wave(60.0 * uv[0], 40.0 * uv[1])];
    let scaled = |uv: [f64; 2]| [k * uv[0], k * uv[1]];
    let mut hard = Vec::new();
    for e in 0..5 {
        for j in 0..8 {
            let uv = edge_point(e, j as f64 / 8.0);
            hard.push(Row::Position { uv: scaled(uv), at: place(uv) });
        }
    }
    let mut soft = Vec::new();
    for j in 1..=12 {
        let uv = edge_point(0, j as f64 / 14.0);
        soft.push((Row::Tangent { uv: scaled(uv), dir: [0.0, 1.0], normal: [0.0, 0.6, 0.8] }, 5.0));
    }
    soft.push((
        Row::Curvature { uv: scaled([0.5, 0.5]), dir: [1.0, 0.0], normal: [0.0, 0.0, 1.0], value: 10.0 / k },
        3.0,
    ));
    let mut data = Vec::new();
    for i in 0..12 {
        for j in 0..12 {
            let uv = [0.3 + 0.4 * i as f64 / 11.0, 0.3 + 0.4 * j as f64 / 11.0];
            data.push(DataPoint { uv: scaled(uv), at: place(uv), weight: 1.0 });
        }
    }
    Problem { domain: [[0.0, k], [0.0, k]], control: [8, 8], hard, soft, data, outline: Vec::new(), smoothing: 1e-3 }
}

#[test]
fn soft_tangent_and_curvature_rows_do_not_depend_on_the_units() {
    let a = net(&solve(&scaled_problem(1.0)).unwrap().surface);
    let b = net(&solve(&scaled_problem(10.0)).unwrap().surface);
    let size = a.iter().flatten().flat_map(|c| c.iter()).fold(0.0_f64, |m, v| m.max(v.abs()));
    for i in 0..8 {
        for j in 0..8 {
            for c in 0..3 {
                let d = (a[i][j][c] - b[i][j][c] / 10.0).abs();
                assert!(
                    d <= 1e-6 * size,
                    "control point {i},{j},{c}: {} against {} (size {size})",
                    a[i][j][c],
                    b[i][j][c] / 10.0
                );
            }
        }
    }
}

/// A coupled problem (G1 on three sides of the pentagon, positions on all five) over an `n × n` net
/// with 7056 data points, 1600 soft rows and `extra` hard interior points, the plane the data lie on
/// being the surface that satisfies every row.
fn coupled_problem(n: usize, extra: usize) -> Problem {
    let mut hard = Vec::new();
    let mut soft = Vec::new();
    let normal = [-0.2, 0.1, 1.0];
    let across = |e: usize| [PENTAGON[(e + 1) % 5][1] - PENTAGON[e][1], PENTAGON[e][0] - PENTAGON[(e + 1) % 5][0]];
    for e in 0..5 {
        for k in 0..2 * n {
            let uv = edge_point(e, k as f64 / (2 * n) as f64);
            hard.push(Row::Position { uv, at: on_plane(uv) });
            if e < 3 {
                hard.push(Row::Tangent { uv, dir: across(e), normal });
            }
        }
        for k in 0..200 {
            let uv = edge_point(e, (k as f64 + 0.5) / 200.0);
            if e < 3 {
                soft.push((Row::Tangent { uv, dir: across(e), normal }, 1e4));
            }
            soft.push((Row::Position { uv, at: on_plane(uv) }, 1e4));
        }
    }
    for k in 0..extra {
        let uv = [0.25 + 0.5 * ((k * 7919) % 1000) as f64 / 1000.0, 0.25 + 0.5 * ((k * 104729) % 997) as f64 / 997.0];
        hard.push(Row::Position { uv, at: on_plane(uv) });
    }
    let mut data = Vec::new();
    for i in 0..84 {
        for j in 0..84 {
            let uv = [0.2 + 0.6 * i as f64 / 83.0, 0.2 + 0.6 * j as f64 / 83.0];
            data.push(DataPoint { uv, at: on_plane(uv), weight: 1.0 });
        }
    }
    Problem { domain: UNIT, control: [n, n], hard, soft, data, outline: PENTAGON.to_vec(), smoothing: 1e-3 }
}

#[test]
fn a_solve_over_the_cost_bound_is_refused_before_it_starts() {
    // 24 × 24 with 3000 hard interior points: tens of seconds of work, estimated over the bound
    let p = coupled_problem(24, 3000);
    assert!(p.is_coupled() && p.unknowns() == 1728);
    assert!(p.estimated_seconds() > MAX_SOLVE_SECONDS, "{}", p.estimated_seconds());
    let started = std::time::Instant::now();
    let err = solve(&p).unwrap_err().to_string();
    assert!(err.contains("limit") && err.contains("hard rows"), "{err}");
    assert!(started.elapsed().as_secs_f64() < 1.0, "the refusal took {:?}", started.elapsed());
    // the measured 26 s and 66 s cases (24 × 24 with 2000 and 9000 hard interior points) are refused
    for extra in [2000, 9000] {
        let p = coupled_problem(24, extra);
        assert!(p.estimated_seconds() > MAX_SOLVE_SECONDS, "{extra}: {}", p.estimated_seconds());
        assert!(solve(&p).is_err());
    }
    // the same net with few rows is accepted by the model, and a decoupled problem of that size too
    assert!(coupled_problem(24, 0).estimated_seconds() <= MAX_SOLVE_SECONDS);
    assert!(coupled_problem(16, 3000).estimated_seconds() <= MAX_SOLVE_SECONDS);
}

#[test]
fn rows_that_can_never_hold_together_leave_a_residual() {
    let uv = [0.5, 0.5];
    let mut hard = pentagon_rows();
    hard.push(Row::Position { uv, at: on_plane(uv) });
    let consistent = solve(&plane_problem(hard.clone())).unwrap();
    assert!(consistent.hard_residual <= 1e-9, "{}", consistent.hard_residual);
    let mut at = on_plane(uv);
    at[2] += 0.4;
    hard.push(Row::Position { uv, at });
    let sol = solve(&plane_problem(hard)).unwrap();
    assert!(sol.hard_residual >= 0.05, "{}", sol.hard_residual);
}

#[test]
fn a_cancelled_solve_returns_the_cancellation() {
    let p = coupled_problem(12, 0);
    let cancel = Cancel::new();
    cancel.cancel();
    assert_eq!(solve_cancellable(&p, &cancel).unwrap_err(), KernelError::Cancelled);
    // a flag set while the solve runs stops it between stages, long before it would have ended
    let big = coupled_problem(24, 0);
    let cancel = Cancel::new();
    let started = std::time::Instant::now();
    let out = std::thread::scope(|scope| {
        scope.spawn(|| {
            std::thread::sleep(std::time::Duration::from_millis(200));
            cancel.cancel();
        });
        solve_cancellable(&big, &cancel)
    });
    assert_eq!(out.unwrap_err(), KernelError::Cancelled);
    assert!(started.elapsed().as_secs_f64() < 15.0, "the cancellation took {:?}", started.elapsed());
}

/// Measurement (ignored): time of one coupled solve against the net size and the hard interior
/// points. `COST_NETS=8,12` picks the nets, `COST_EXTRA` the hard points; run with
/// `-- --ignored --nocapture`. Measured on the development laptop (aarch64, `ci` profile, one
/// thread; the model [`solve_seconds`] is fitted to these):
///
/// | net | unknowns | hard scalar rows | seconds | peak memory |
/// |---|---|---|---|---|
/// | 8 × 8 | 192 | 184 | 0.04 | 10 MB |
/// | 12 × 12 | 432 | 338 | 0.23 | 30 MB |
/// | 16 × 16 | 768 | 455 | 0.8 to 1.3 | 72 MB |
/// | 20 × 20 | 1200 | 594 | 3.0 | 150 MB |
/// | 24 × 24 | 1728 | 714 | 4.0 to 5.2 | 278 MB |
/// | 16 × 16 | 768 | 552 (2256 rows) | 3.7 to 5.1 | 104 MB |
/// | 24 × 24 | 1728 | 1136 (2384 rows) | 26 | 400 MB |
/// | 16 × 16 | 768 | 552 (9256 rows) | 11.5 | 105 MB |
/// | 24 × 24 | 1728 | 1131 (9384 rows) | 66 | 433 MB |
/// | 24 × 24 | 1728 | 714 (864 rows, the largest the bound accepts) | 3.99 (estimate 5.7) | about 300 MB |
#[test]
#[ignore = "measurement"]
fn cost_of_coupled_solves() {
    let nets: Vec<usize> = std::env::var("COST_NETS")
        .unwrap_or_else(|_| "8,12,16,24".into())
        .split(',')
        .filter_map(|s| s.parse().ok())
        .collect();
    let extra: usize = std::env::var("COST_EXTRA").ok().and_then(|s| s.parse().ok()).unwrap_or(0);
    for n in nets {
        let p = coupled_problem(n, extra);
        let est = p.estimated_seconds();
        let t = std::time::Instant::now();
        // measured with the bound lifted: the estimate is printed next to the time
        let out = solve(&p);
        println!(
            "COST net {n}x{n} unknowns {} hard rows {} soft {} data {}: {:.2} s (estimate {est:.1} s) {:?}",
            p.unknowns(),
            p.hard_scalar_rows(),
            p.soft.len(),
            p.data.len(),
            t.elapsed().as_secs_f64(),
            out.map(|s| s.rank).map_err(|e| e.to_string())
        );
    }
}

/// Measurement (ignored): the largest net the bound accepts, 24 × 24, with as many hard interior
/// points as it allows, must solve within the bound. Measured: see the assertion message.
#[test]
#[ignore = "measurement"]
fn the_largest_accepted_net_solves_within_the_bound() {
    let mut extra = 0;
    while coupled_problem(24, extra + 100).estimated_seconds() <= MAX_SOLVE_SECONDS {
        extra += 100;
    }
    let p = coupled_problem(24, extra);
    let est = p.estimated_seconds();
    let started = std::time::Instant::now();
    let sol = solve(&p).unwrap();
    let took = started.elapsed().as_secs_f64();
    println!(
        "COST largest accepted: 24 x 24, {} hard rows, estimate {est:.1} s, took {took:.2} s, rank {}",
        p.hard_scalar_rows(),
        sol.rank
    );
    assert!(took <= 1.5 * MAX_SOLVE_SECONDS, "took {took:.1} s against an estimate of {est:.1} s");
}
