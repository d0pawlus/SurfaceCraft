//! The cost bound and the cancellation of the driver: a call over the bound stops or is refused
//! as designed, a cancelled call produces nothing, and (ignored, a measurement) the time of the
//! largest accepted net.
use super::tests::{Rng, cylinder_hole, on_cylinder};
use super::*;
use crate::vpatch::{MAX_SOLVE_SECONDS, solve_seconds};
use std::time::{Duration, Instant};

/// The five-sided hole of `cylinder_hole` with `n` noisy scan points and `hard` points the patch
/// must pass through (every one a hard position row of three scalar rows in the coupled solve).
fn loaded_hole(continuity: Continuity, n: usize, hard: usize) -> Request {
    let mut req = cylinder_hole(continuity);
    let mut rng = Rng(11);
    let point = |noise: f64, rng: &mut Rng| loop {
        let (s, z) = (6.0 * (2.0 * rng.next() - 1.0), 6.0 * (2.0 * rng.next() - 1.0));
        if s * s + z * z <= 25.0 {
            let p = on_cylinder(s, z);
            return [p[0] + noise * rng.normal(), p[1] + noise * rng.normal(), p[2] + noise * rng.normal()];
        }
    };
    req.data = (0..n).map(|_| point(0.03, &mut rng)).collect();
    req.hard_points = (0..hard).map(|_| point(0.0, &mut rng)).collect();
    req
}

#[test]
fn a_start_net_over_the_cost_bound_is_refused_before_any_solve() {
    // 24 × 24 with G1 sides and 1500 hard points: 4500+ hard scalar rows over 1728 unknowns
    let mut req = loaded_hole(Continuity::G1, 200, 1500);
    req.control = Some([24, 24]);
    req.max_control = 24;
    assert!(solve_seconds(3 * 24 * 24, 4500) > MAX_SOLVE_SECONDS);
    let started = Instant::now();
    let err = fit_patch(&req).unwrap_err().to_string();
    assert!(err.contains("per solve") && err.contains("limit"), "{err}");
    assert!(started.elapsed() < Duration::from_secs(10), "the refusal took {:?}", started.elapsed());
}

#[test]
fn growth_stops_at_the_cost_bound_and_the_report_says_so() {
    // a tolerance the noise never allows makes the net grow; with 1500 hard scalar rows a net of
    // 24 × 24 is estimated over the bound while 16 × 16 is far under it
    let mut req = loaded_hole(Continuity::G1, 1200, 500);
    req.tolerance = 1e-4;
    req.max_control = 24;
    req.control = Some([16, 16]);
    assert!(solve_seconds(3 * 16 * 16, 1500) <= MAX_SOLVE_SECONDS);
    assert!(solve_seconds(3 * 24 * 24, 1500) > MAX_SOLVE_SECONDS);
    let (patch, rounds) = fit_patch_rounds(&req).unwrap();
    let r = &patch.report;
    assert!(r.cost_limited, "{r:?}");
    assert!(r.control.iter().all(|&n| n <= 16), "{:?}", r.control);
    // the bound is on the work estimate, not on the wall clock (a loaded runner is slower than the model)
    let estimate = |t: &RoundTrace| solve_seconds(3 * t.net[0] * t.net[1], t.hard_rows);
    assert!(rounds.iter().all(|t| estimate(t) <= MAX_SOLVE_SECONDS), "{rounds:?}");
    assert!(r.warnings.iter().any(|w| w.contains("stopped at") && w.contains("limit")), "{:?}", r.warnings);
    assert!(r.rms.is_finite() && r.max.is_finite());
}

#[test]
fn a_cancelled_fit_returns_the_cancellation_and_nothing_else() {
    let req = loaded_hole(Continuity::G1, 3000, 0);
    let cancel = Cancel::new();
    cancel.cancel();
    assert_eq!(fit_patch_cancellable(&req, &cancel).unwrap_err(), KernelError::Cancelled);
}

#[test]
fn a_fit_cancelled_while_it_runs_ends_promptly() {
    // G1 on 16 × 16 with 6000 data points: about a second per round
    let mut req = loaded_hole(Continuity::G1, 6000, 0);
    req.control = Some([16, 16]);
    let cancel = Cancel::new();
    let started = Instant::now();
    let out = std::thread::scope(|scope| {
        scope.spawn(|| {
            std::thread::sleep(Duration::from_millis(150));
            cancel.cancel();
        });
        fit_patch_cancellable(&req, &cancel)
    });
    assert_eq!(out.unwrap_err(), KernelError::Cancelled);
    assert!(started.elapsed() < Duration::from_secs(60), "the cancellation took {:?}", started.elapsed());
}

/// Measurement (ignored): rounds of a G1 fit against the control net, with the solve time of each.
/// `COST_NETS=8,12` picks the nets, `COST_HARD` the hard points, `COST_G2` G2 sides, `COST_TOL` the
/// tolerance (1e-4 keeps the rounds going); run with `-- --ignored --nocapture`.
#[test]
#[ignore = "measurement"]
fn cost_of_full_fits() {
    let nets: Vec<usize> = std::env::var("COST_NETS")
        .unwrap_or_else(|_| "8,12,16,24".into())
        .split(',')
        .filter_map(|s| s.parse().ok())
        .collect();
    let hard: usize = std::env::var("COST_HARD").ok().and_then(|s| s.parse().ok()).unwrap_or(0);
    for n in nets {
        let g2 = std::env::var("COST_G2").is_ok();
        let mut req = loaded_hole(if g2 { Continuity::G2 } else { Continuity::G1 }, 7000, hard);
        if let Ok(t) = std::env::var("COST_TOL") {
            req.tolerance = t.parse().unwrap_or(0.2);
        }
        req.control = Some([n, n]);
        req.max_control = n;
        let started = Instant::now();
        let out = fit_patch_rounds(&req);
        let total = started.elapsed().as_secs_f64();
        match out {
            Ok((p, rounds)) => println!(
                "COST fit net {n}x{n}: {total:.2} s in {} rounds, solves {:?} s, hard rows {}, rms {:.3}",
                rounds.len(),
                rounds.iter().map(|t| (t.solve_secs * 100.0).round() / 100.0).collect::<Vec<_>>(),
                rounds.last().map_or(0, |t| t.hard_rows),
                p.report.rms
            ),
            Err(e) => println!("COST fit net {n}x{n}: {total:.2} s, refused: {e}"),
        }
    }
}
