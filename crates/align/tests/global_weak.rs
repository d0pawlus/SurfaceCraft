//! P9-M3 review: a scan held by a single pair is never `aligned`, and a pair that contradicts the
//! others is left out with its misfit visible. The pair solver is replaced through
//! `align_global_with`: whole copies of the bracket give real, `unique` pair poses (the M2 gate),
//! and a chosen pair is forged to a pose that is wrong by 8 mm with full confidence, the way
//! `pair_coarse` has been seen to return a wrong pose as `unique`.
use surfacecraft_align::global::{EdgeStatus, GlobalIssue, align_global_with};
use surfacecraft_align::{
    AlignError, GlobalOptions, GlobalResult, GlobalVerdict, PairResult, PairVerdict, pair_coarse,
};
use surfacecraft_geom::{Cancel, Mesh, Point3, Rng, Transform, Vec3, synth};

const FORGED_SHIFT_MM: f64 = 8.0;

fn scan(block: usize, seed: u64) -> Mesh {
    let mut m = synth::cuboid(Point3::origin(), Vec3::new(60.0, 40.0, 20.0), block);
    m.append(&synth::cylinder(Point3::new(45.0, 12.0, 20.0), Vec3::z(), 6.0, 10.0, 48, 6));
    m.append(&synth::sphere(Point3::new(15.0, 28.0, 20.0), 5.0, 32, 16));
    synth::add_noise(&mut m, 0.05, &mut Rng::new(seed));
    m
}

fn poses() -> [Transform; 4] {
    [
        Transform::identity(),
        Transform::rotation_axis_angle(&Vec3::new(0.3, -0.8, 0.5), 2.4)
            .then_after(&Transform::translation(Vec3::new(90.0, -40.0, 30.0))),
        Transform::rotation_axis_angle(&Vec3::new(-0.6, 0.1, 0.9), 1.1)
            .then_after(&Transform::translation(Vec3::new(-70.0, 20.0, 60.0))),
        Transform::rotation_axis_angle(&Vec3::new(0.1, 0.9, -0.2), 0.7)
            .then_after(&Transform::translation(Vec3::new(20.0, 80.0, -50.0))),
    ]
}

/// What the pair solver does for a pair: the real alignment, a forged one, or a failure.
#[derive(Clone, Copy, PartialEq)]
enum Pair {
    Real,
    Forged,
    Fails,
}

fn run(n: usize, how: impl Fn(usize, usize) -> Pair + Sync) -> GlobalResult {
    let poses = poses();
    let meshes: Vec<Mesh> = (0..n).map(|k| scan(12 + 3 * k, 1 + k as u64).transformed(&poses[k])).collect();
    let refs: Vec<&Mesh> = meshes.iter().collect();
    let solver =
        |i: usize, j: usize, moving: &Mesh, fixed: &Mesh, o: &_, c: &Cancel| -> Result<PairResult, AlignError> {
            match how(i, j) {
                Pair::Fails => Err(AlignError::NotEnough("forbidden by the test".into())),
                Pair::Real => pair_coarse(moving, fixed, o, c),
                Pair::Forged => {
                    let real = pair_coarse(moving, fixed, o, c)?;
                    // along the part's own x axis as the fixed scan sees it: the faces slide on themselves
                    let shift = Transform::translation(poses[i].rotation * Vec3::new(FORGED_SHIFT_MM, 0.0, 0.0));
                    Ok(PairResult {
                        transform: shift.then_after(&real.transform),
                        verdict: PairVerdict::Unique,
                        confidence: 1.0,
                        ..real
                    })
                }
            }
        };
    align_global_with(&refs, &GlobalOptions::default(), &Cancel::new(), &solver).unwrap()
}

fn issues(r: &GlobalResult) -> String {
    format!("{:?}", r.issues)
}

/// (a) A leaf that hangs on a forged `unique` pair: placed, but nothing claims it is confirmed.
#[test]
fn a_scan_on_a_forged_unique_pair_is_flagged_and_the_verdict_is_not_aligned() {
    let r = run(3, |i, j| match (i, j) {
        (0, 2) => Pair::Forged,
        (1, 2) => Pair::Fails,
        _ => Pair::Real,
    });
    eprintln!("verdict {:?}, issues {}", r.verdict, issues(&r));
    assert_ne!(r.verdict, GlobalVerdict::Aligned);
    assert!(r.issues.contains(&GlobalIssue::WeaklyHeld));
    assert!(r.scans[2].placed && !r.scans[2].held_by_loop, "the leaf is placed and flagged");
    assert!(r.scans[0].held_by_loop, "the fixed scan counts as held");
}

/// (b) Two scans are one pair: never confirmed by a loop.
#[test]
fn two_scans_are_one_unconfirmed_pair() {
    let r = run(2, |_, _| Pair::Real);
    eprintln!("verdict {:?}, issues {}, pair confidence {:.2}", r.verdict, issues(&r), r.edges[0].confidence);
    assert_eq!(r.edges[0].status, EdgeStatus::Used);
    assert_eq!(r.verdict, GlobalVerdict::Partial);
    assert_eq!(r.issues, vec![GlobalIssue::WeaklyHeld]);
    assert!(r.scans[1].placed && !r.scans[1].held_by_loop);
}

/// (c) A chain 0 - 1 - 2 has two bridges: both scans are weakly held.
#[test]
fn a_chain_has_two_bridges_and_both_scans_are_weakly_held() {
    let r = run(3, |i, j| if (i, j) == (0, 2) { Pair::Fails } else { Pair::Real });
    assert_eq!(r.verdict, GlobalVerdict::Partial);
    assert!(r.issues.contains(&GlobalIssue::WeaklyHeld));
    assert!(!r.scans[1].held_by_loop && !r.scans[2].held_by_loop);
}

/// A triangle of real pairs holds every scan: aligned, nothing weak.
#[test]
fn a_triangle_of_real_pairs_holds_every_scan() {
    let r = run(3, |_, _| Pair::Real);
    eprintln!("verdict {:?}, issues {}", r.verdict, issues(&r));
    assert_eq!(r.verdict, GlobalVerdict::Aligned, "{:?}", r.issues);
    assert!(r.scans.iter().all(|s| s.held_by_loop));
    assert!(r.issues.is_empty());
}

/// A forged pair among real ones is left out by the vote, and its misfit at the final poses is
/// reported; the counts say how many were used and left out; the vote's margin is reported.
#[test]
fn a_rejected_pair_reports_its_misfit_and_the_counts() {
    let r = run(4, |i, j| if (i, j) == (1, 3) { Pair::Forged } else { Pair::Real });
    for e in &r.edges {
        eprintln!("  {}-{} {:?} {:?} {:?}", e.fixed, e.moving, e.verdict, e.status, e.misfit_after);
    }
    let forged = r.edges.iter().find(|e| (e.fixed, e.moving) == (1, 3)).unwrap();
    let after = forged.misfit_after.unwrap();
    eprintln!(
        "forged pair: {:?}, misfit after {:.3} mm {:.3} deg; used {} rejected {} margin {:?}; verdict {:?} {}",
        forged.status,
        after.rms_mm,
        after.angle_degrees,
        r.edges_used,
        r.edges_rejected,
        r.consensus_margin,
        r.verdict,
        issues(&r)
    );
    assert_eq!(forged.status, EdgeStatus::Rejected);
    assert!(after.rms_mm >= 2.0, "the contradiction must be visible: {}", after.rms_mm);
    assert_eq!((r.edges_used, r.edges_rejected), (5, 1));
    assert!(r.consensus_margin.is_some_and(|m| m >= 1), "{:?}", r.consensus_margin);
    assert_eq!(r.verdict, GlobalVerdict::Aligned, "{:?}", r.issues);
    assert!(r.issues.contains(&GlobalIssue::EdgeRejected));
}
