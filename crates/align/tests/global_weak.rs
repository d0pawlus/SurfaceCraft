//! P9-M3 review: a scan held by a single pair is never `aligned`, and a pair that contradicts the
//! others is left out with its misfit visible. The pair solver is replaced through
//! `align_global_with`: whole copies of the bracket give real, `unique` pair poses (the M2 gate),
//! and a chosen pair is forged to a pose that is wrong by 8 mm with full confidence, the way
//! `pair_coarse` has been seen to return a wrong pose as `unique`.
use std::collections::HashMap;
use std::sync::{Mutex, OnceLock};
use surfacecraft_align::global::{EdgeStatus, GlobalIssue, align_global_with};
use surfacecraft_align::{
    AlignError, GlobalOptions, GlobalResult, GlobalVerdict, PairOptions, PairResult, PairVerdict, pair_coarse,
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
    /// Wrong by 8 mm along the part's own x axis, `unique`, confidence 1.0.
    Forged,
    /// The real pose with a residual of 5 mm (about 7 % of the diagonal): weak evidence.
    WeakEvidence,
    /// Wrong consistently with the forged pair (0, 2): for (1, 2) the pose that closes the triangle.
    ConsistentlyWrong,
    Fails,
}

/// The real pair alignments, computed once per pair for all the tests of this file (the scans
/// depend only on their index).
fn real_pair(i: usize, j: usize, meshes: &[Mesh], o: &PairOptions, c: &Cancel) -> Result<PairResult, AlignError> {
    static MEMO: OnceLock<Mutex<HashMap<(usize, usize), PairResult>>> = OnceLock::new();
    let memo = MEMO.get_or_init(|| Mutex::new(HashMap::new()));
    if let Some(r) = memo.lock().ok().and_then(|m| m.get(&(i, j)).cloned()) {
        return Ok(r);
    }
    let r = pair_coarse(&meshes[j], &meshes[i], o, c)?;
    if let Ok(mut m) = memo.lock() {
        m.insert((i, j), r.clone());
    }
    Ok(r)
}

fn run(n: usize, how: impl Fn(usize, usize) -> Pair + Sync) -> GlobalResult {
    let poses = poses();
    let meshes: Vec<Mesh> = (0..n).map(|k| scan(12 + 3 * k, 1 + k as u64).transformed(&poses[k])).collect();
    let refs: Vec<&Mesh> = meshes.iter().collect();
    let forged = |i: usize, j: usize, o: &PairOptions, c: &Cancel| -> Result<PairResult, AlignError> {
        let real = real_pair(i, j, &meshes, o, c)?;
        // along the part's own x axis as the fixed scan sees it: the faces slide on themselves
        let shift = Transform::translation(poses[i].rotation * Vec3::new(FORGED_SHIFT_MM, 0.0, 0.0));
        Ok(PairResult {
            transform: shift.then_after(&real.transform),
            verdict: PairVerdict::Unique,
            confidence: 1.0,
            ..real
        })
    };
    let solver =
        |i: usize, j: usize, _: &Mesh, _: &Mesh, o: &PairOptions, c: &Cancel| -> Result<PairResult, AlignError> {
            match how(i, j) {
                Pair::Fails => Err(AlignError::NotEnough("forbidden by the test".into())),
                Pair::Real => real_pair(i, j, &meshes, o, c),
                Pair::Forged => forged(i, j, o, c),
                Pair::WeakEvidence => Ok(PairResult { rms: 5.0, ..real_pair(i, j, &meshes, o, c)? }),
                Pair::ConsistentlyWrong => {
                    // scan j -> scan 0 (wrong), then scan 0 -> scan i by the real pose of the pair (0, i)
                    let to_zero = forged(0, j, o, c)?;
                    let zero_to_i = real_pair(0, i, &meshes, o, c)?.transform;
                    Ok(PairResult { transform: zero_to_i.inverse().then_after(&to_zero.transform), ..to_zero })
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

/// A pair that is `unique` and wrong by 8 mm among real ones: the vote leaves it out and reports
/// its misfit and the counts. The graph has 6 pairs, so the vote must win by 2; it wins by `margin`
/// (printed) and the verdict follows the rule.
#[test]
fn a_rejected_pair_reports_its_misfit_and_the_counts() {
    let r = run(4, |i, j| if (i, j) == (1, 3) { Pair::Forged } else { Pair::Real });
    for e in &r.edges {
        eprintln!(
            "  {}-{} {:?} {:?} rms/diagonal {:.5} strong {}",
            e.fixed, e.moving, e.verdict, e.status, e.rms_per_diagonal, e.strong_evidence
        );
    }
    let forged = r.edges.iter().find(|e| (e.fixed, e.moving) == (1, 3)).unwrap();
    let after = forged.misfit_after.unwrap();
    eprintln!(
        "forged pair: {:?}, misfit after {:.3} mm; used {} rejected {} margin {:?} of {} trials; verdict {:?} {}",
        forged.status,
        after.rms_mm,
        r.edges_used,
        r.edges_rejected,
        r.consensus_margin,
        r.consensus_trials,
        r.verdict,
        issues(&r)
    );
    assert_eq!(forged.status, EdgeStatus::Rejected);
    assert!((after.rms_mm - FORGED_SHIFT_MM).abs() <= 0.1, "the contradiction must be visible: {}", after.rms_mm);
    assert_eq!((r.edges_used, r.edges_rejected), (5, 1));
    assert!(r.consensus_trials > 1);
    assert!(r.issues.contains(&GlobalIssue::EdgeRejected));
    let wins_clearly = r.consensus_margin.is_some_and(|m| m >= 2);
    assert_eq!(r.verdict == GlobalVerdict::Aligned, wins_clearly, "{:?}", r.issues);
}

/// A loop of pairs that agree with each other, one of which has weak evidence of its own (a
/// residual of 5 mm), confirms nothing: the scans are weakly held.
#[test]
fn a_loop_with_a_weak_pair_does_not_hold_its_scans() {
    let r = run(3, |i, j| if (i, j) == (0, 2) { Pair::WeakEvidence } else { Pair::Real });
    for e in &r.edges {
        eprintln!("  {}-{} rms/diagonal {:.5} strong {}", e.fixed, e.moving, e.rms_per_diagonal, e.strong_evidence);
    }
    assert!(r.edges.iter().any(|e| !e.strong_evidence) && r.edges.iter().any(|e| e.strong_evidence));
    assert_eq!(r.verdict, GlobalVerdict::Partial);
    assert!(r.issues.contains(&GlobalIssue::WeaklyHeld));
    assert!(r.scans[1].placed && r.scans[2].placed && !r.scans[1].held_by_loop && !r.scans[2].held_by_loop);
    assert!(r.scans[1].weak_pair.is_some());
}

/// THE LIMIT, documented: C placed against both A and B by the same wrong pose (here 8 mm off,
/// the way a symmetry of the part would do it) makes a triangle that closes, every pair carries
/// strong evidence of its own (the wrong pose fits the faces that coincide at noise level), and
/// nothing in the data tells the loop from a right one: the verdict is `aligned`. It is the pair
/// alignment that must not call such a pose `unique` (see the reproduction
/// `pair_coarse_false_unique_repro` in the engine tests); here `aligned` means "the pair poses
/// agree with each other", and the command's note and the guide say so.
#[test]
fn two_consistently_wrong_pairs_close_a_loop_and_are_aligned_a_documented_limit() {
    let r = run(3, |i, j| match (i, j) {
        (0, 1) => Pair::Real,
        _ => Pair::ConsistentlyWrong,
    });
    eprintln!("verdict {:?}, issues {}, C held {}", r.verdict, issues(&r), r.scans[2].held_by_loop);
    assert_eq!(r.verdict, GlobalVerdict::Aligned, "if this changes, a rule now catches the limit: update the guide");
    assert!(r.scans[2].held_by_loop);
}
